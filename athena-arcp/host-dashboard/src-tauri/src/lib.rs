use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use arcp_dashboard::{
    format_signal_value, ControlClient, DashboardState, ManifestItem, TelemetrySubscription,
};
use serde::Serialize;
use tauri::{PhysicalPosition, PhysicalSize, State, Window};

const CONTROL_RECONNECT_INTERVAL: Duration = Duration::from_millis(1000);
const CONTROL_HEALTHCHECK_INTERVAL: Duration = Duration::from_millis(1500);
const SERVER_STATS_REFRESH_INTERVAL: Duration = Duration::from_millis(500);

struct AppRuntime {
    session: Mutex<Option<Session>>,
    presentation_mode: AtomicBool,
    dock_mode: AtomicBool,
}

impl Default for AppRuntime {
    fn default() -> Self {
        Self {
            session: Mutex::new(None),
            presentation_mode: AtomicBool::new(false),
            dock_mode: AtomicBool::new(false),
        }
    }
}

struct Session {
    host: String,
    control_port: u16,
    udp_port: u16,
    running: Arc<AtomicBool>,
    state: Arc<Mutex<DashboardState>>,
    control: Arc<Mutex<ControlRuntime>>,
    local_stats: Mutex<ProcessStatsSampler>,
    server_stats: Mutex<ServerStatsCache>,
    telemetry_handle: Option<JoinHandle<()>>,
    control_handle: Option<JoinHandle<()>>,
}

impl Session {
    fn stop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Ok(mut guard) = self.control.lock() {
            if let Some(client) = guard.client.as_mut() {
                let _ = client.unsubscribe(self.udp_port);
            }
            guard.client = None;
            guard.connected = false;
            guard.status = format!("disconnected from {}:{}", self.host, self.control_port);
        }
        if let Some(handle) = self.telemetry_handle.take() {
            let _ = handle.join();
        }
        if let Some(handle) = self.control_handle.take() {
            let _ = handle.join();
        }
    }
}

struct ControlRuntime {
    client: Option<ControlClient>,
    connected: bool,
    status: String,
}

#[derive(Serialize)]
struct ConnectInfo {
    connected: bool,
    host: String,
    control_port: u16,
    udp_port: u16,
    signal_count: usize,
}

#[derive(Serialize)]
struct DashboardSnapshot {
    connected: bool,
    status: String,
    signal_count: usize,
    update_count: u64,
    uptime_ms: u64,
    server_cpu_percent: Option<f32>,
    server_rss_bytes: Option<u64>,
    host_cpu_percent: Option<f32>,
    host_rss_bytes: Option<u64>,
    signals: Vec<SignalRow>,
}

#[derive(Serialize)]
struct DashboardDelta {
    connected: bool,
    status: String,
    signal_count: usize,
    update_count: u64,
    uptime_ms: u64,
    server_cpu_percent: Option<f32>,
    server_rss_bytes: Option<u64>,
    host_cpu_percent: Option<f32>,
    host_rss_bytes: Option<u64>,
    manifest_revision: u64,
    full_snapshot: bool,
    signals: Vec<SignalRow>,
}

#[derive(Serialize)]
struct SignalRow {
    signal_id: u16,
    signal_type: String,
    kind: String,
    access: String,
    policy: String,
    durability: String,
    path: String,
    value: String,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct WindowModeSnapshot {
    presentation_mode: bool,
    dock_mode: bool,
}

#[derive(Default)]
struct ProcessStatsSampler {
    prev_proc_jiffies: Option<u64>,
    prev_total_jiffies: Option<u64>,
    cpu_scale: f64,
}

impl ProcessStatsSampler {
    fn new() -> Self {
        Self {
            prev_proc_jiffies: None,
            prev_total_jiffies: None,
            cpu_scale: std::thread::available_parallelism()
                .map(|v| v.get() as f64)
                .unwrap_or(1.0),
        }
    }

    fn sample(&mut self) -> (Option<f32>, Option<u64>) {
        let rss = read_rss_bytes();
        let proc_jiffies = read_process_jiffies();
        let total_jiffies = read_total_jiffies();

        let mut cpu_percent = None;
        if let (Some(proc_now), Some(total_now)) = (proc_jiffies, total_jiffies) {
            if let (Some(proc_prev), Some(total_prev)) =
                (self.prev_proc_jiffies, self.prev_total_jiffies)
            {
                let delta_proc = proc_now.saturating_sub(proc_prev);
                let delta_total = total_now.saturating_sub(total_prev);
                if delta_total > 0 {
                    let cpu = (delta_proc as f64 / delta_total as f64) * self.cpu_scale * 100.0;
                    cpu_percent = Some(cpu as f32);
                }
            }
            self.prev_proc_jiffies = Some(proc_now);
            self.prev_total_jiffies = Some(total_now);
        }

        (cpu_percent, rss)
    }
}

struct ServerStatsCache {
    last_refresh: Instant,
    unsupported: bool,
    cpu_percent: Option<f32>,
    rss_bytes: Option<u64>,
}

impl ServerStatsCache {
    fn new() -> Self {
        Self {
            last_refresh: Instant::now() - SERVER_STATS_REFRESH_INTERVAL,
            unsupported: false,
            cpu_percent: None,
            rss_bytes: None,
        }
    }

    fn maybe_refresh(&mut self, control: &mut ControlRuntime) {
        if self.unsupported || self.last_refresh.elapsed() < SERVER_STATS_REFRESH_INTERVAL {
            return;
        }
        self.last_refresh = Instant::now();

        let Some(client) = control.client.as_mut() else {
            self.cpu_percent = None;
            self.rss_bytes = None;
            return;
        };

        match client.server_stats() {
            Ok((cpu_percent, rss_bytes)) => {
                self.cpu_percent = cpu_percent;
                self.rss_bytes = rss_bytes;
            }
            Err(err) if err.kind() == io::ErrorKind::Unsupported => {
                self.unsupported = true;
                self.cpu_percent = None;
                self.rss_bytes = None;
            }
            Err(err) => {
                control.client = None;
                control.connected = false;
                control.status = format!("STATS failed: {err}; reconnecting...");
                self.cpu_percent = None;
                self.rss_bytes = None;
            }
        }
    }

    fn snapshot(&self) -> (Option<f32>, Option<u64>) {
        (self.cpu_percent, self.rss_bytes)
    }
}

#[tauri::command]
fn connect_arcp(
    runtime: State<'_, AppRuntime>,
    host: String,
    control_port: u16,
) -> Result<ConnectInfo, String> {
    let host = host.trim().to_string();
    if host.is_empty() {
        return Err(String::from("host must not be empty"));
    }

    let mut session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;

    if let Some(mut existing) = session_slot.take() {
        existing.stop();
    }

    let mut control = ControlClient::connect(&host, control_port)
        .map_err(|err| format!("connect failed: {err}"))?;
    control
        .ping()
        .map_err(|err| format!("PING failed: {err}"))?;
    let manifest = control
        .manifest()
        .map_err(|err| format!("manifest failed: {err}"))?;

    let telemetry =
        TelemetrySubscription::bind_any().map_err(|err| format!("bind UDP failed: {err}"))?;
    let udp_port = telemetry
        .local_port()
        .map_err(|err| format!("read UDP local port failed: {err}"))?;
    control
        .subscribe(udp_port)
        .map_err(|err| format!("subscribe failed: {err}"))?;

    let signal_count = manifest.len();
    let state = Arc::new(Mutex::new(DashboardState::new(manifest)));
    let running = Arc::new(AtomicBool::new(true));
    let telemetry_handle = telemetry
        .spawn(Arc::clone(&state), Arc::clone(&running))
        .map_err(|err| format!("telemetry worker failed: {err}"))?;

    let control_runtime = Arc::new(Mutex::new(ControlRuntime {
        client: Some(control),
        connected: true,
        status: format!("connected to {host}:{control_port} (udp {udp_port})"),
    }));

    let control_handle = spawn_control_worker(
        Arc::clone(&running),
        Arc::clone(&state),
        Arc::clone(&control_runtime),
        host.clone(),
        control_port,
        udp_port,
    )
    .map_err(|err| format!("control worker failed: {err}"))?;

    *session_slot = Some(Session {
        host: host.clone(),
        control_port,
        udp_port,
        running,
        state,
        control: control_runtime,
        local_stats: Mutex::new(ProcessStatsSampler::new()),
        server_stats: Mutex::new(ServerStatsCache::new()),
        telemetry_handle: Some(telemetry_handle),
        control_handle: Some(control_handle),
    });

    Ok(ConnectInfo {
        connected: true,
        host,
        control_port,
        udp_port,
        signal_count,
    })
}

#[tauri::command]
fn disconnect_arcp(runtime: State<'_, AppRuntime>) -> Result<(), String> {
    let mut session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;
    if let Some(mut session) = session_slot.take() {
        session.stop();
    }
    Ok(())
}

#[tauri::command]
fn dashboard_snapshot(runtime: State<'_, AppRuntime>) -> Result<DashboardSnapshot, String> {
    let session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;
    let session = session_slot
        .as_ref()
        .ok_or_else(|| String::from("not connected"))?;

    let (connected, status, server_cpu_percent, server_rss_bytes) = {
        let mut control = session
            .control
            .lock()
            .map_err(|_| String::from("control lock poisoned"))?;
        let mut server_stats = session
            .server_stats
            .lock()
            .map_err(|_| String::from("server stats lock poisoned"))?;
        server_stats.maybe_refresh(&mut control);
        let (server_cpu_percent, server_rss_bytes) = server_stats.snapshot();
        (
            control.connected,
            control.status.clone(),
            server_cpu_percent,
            server_rss_bytes,
        )
    };

    let (signal_count, update_count, uptime_ms, signals) = {
        let guard = session
            .state
            .lock()
            .map_err(|_| String::from("state lock poisoned"))?;
        let mut rows = Vec::with_capacity(guard.descriptors().len());
        for descriptor in guard.descriptors() {
            rows.push(build_signal_row(&guard, descriptor));
        }
        (
            guard.descriptors().len(),
            guard.update_count(),
            guard.uptime_ms(),
            rows,
        )
    };

    let (host_cpu_percent, host_rss_bytes) = {
        let mut stats = session
            .local_stats
            .lock()
            .map_err(|_| String::from("local stats lock poisoned"))?;
        stats.sample()
    };

    Ok(DashboardSnapshot {
        connected,
        status,
        signal_count,
        update_count,
        uptime_ms,
        server_cpu_percent,
        server_rss_bytes,
        host_cpu_percent,
        host_rss_bytes,
        signals,
    })
}

#[tauri::command]
fn dashboard_delta(
    runtime: State<'_, AppRuntime>,
    since_update_count: Option<u64>,
    known_manifest_revision: Option<u64>,
) -> Result<DashboardDelta, String> {
    let session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;
    let session = session_slot
        .as_ref()
        .ok_or_else(|| String::from("not connected"))?;

    let (connected, status, server_cpu_percent, server_rss_bytes) = {
        let mut control = session
            .control
            .lock()
            .map_err(|_| String::from("control lock poisoned"))?;
        let mut server_stats = session
            .server_stats
            .lock()
            .map_err(|_| String::from("server stats lock poisoned"))?;
        server_stats.maybe_refresh(&mut control);
        let (server_cpu_percent, server_rss_bytes) = server_stats.snapshot();
        (
            control.connected,
            control.status.clone(),
            server_cpu_percent,
            server_rss_bytes,
        )
    };

    let (signal_count, update_count, uptime_ms, manifest_revision, full_snapshot, signals) = {
        let guard = session
            .state
            .lock()
            .map_err(|_| String::from("state lock poisoned"))?;
        let signal_count = guard.descriptors().len();
        let update_count = guard.update_count();
        let uptime_ms = guard.uptime_ms();
        let manifest_revision = guard.manifest_revision();

        let full_snapshot = match since_update_count {
            Some(since) => {
                since > update_count || known_manifest_revision != Some(manifest_revision)
            }
            None => true,
        };

        if full_snapshot {
            let mut rows = Vec::with_capacity(signal_count);
            for descriptor in guard.descriptors() {
                rows.push(build_signal_row(&guard, descriptor));
            }
            (
                signal_count,
                update_count,
                uptime_ms,
                manifest_revision,
                full_snapshot,
                rows,
            )
        } else if let Some(since) = since_update_count {
            let updated_ids = guard.updated_signal_ids_since(since);
            let mut rows = Vec::with_capacity(updated_ids.len());
            for signal_id in updated_ids {
                if let Some(descriptor) = guard.descriptor_for(signal_id) {
                    rows.push(build_signal_row(&guard, descriptor));
                }
            }
            (
                signal_count,
                update_count,
                uptime_ms,
                manifest_revision,
                full_snapshot,
                rows,
            )
        } else {
            (
                signal_count,
                update_count,
                uptime_ms,
                manifest_revision,
                full_snapshot,
                Vec::new(),
            )
        }
    };

    let (host_cpu_percent, host_rss_bytes) = {
        let mut stats = session
            .local_stats
            .lock()
            .map_err(|_| String::from("local stats lock poisoned"))?;
        stats.sample()
    };

    Ok(DashboardDelta {
        connected,
        status,
        signal_count,
        update_count,
        uptime_ms,
        server_cpu_percent,
        server_rss_bytes,
        host_cpu_percent,
        host_rss_bytes,
        manifest_revision,
        full_snapshot,
        signals,
    })
}

#[tauri::command]
fn set_signal_value(
    runtime: State<'_, AppRuntime>,
    signal_id: u16,
    value_raw: String,
) -> Result<(), String> {
    let session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;
    let session = session_slot
        .as_ref()
        .ok_or_else(|| String::from("not connected"))?;
    with_control_client(session, "SET", |client| {
        client.set_value(signal_id, &value_raw)
    })
}

#[tauri::command]
fn trigger_action(runtime: State<'_, AppRuntime>, signal_id: u16) -> Result<(), String> {
    let session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;
    let session = session_slot
        .as_ref()
        .ok_or_else(|| String::from("not connected"))?;
    with_control_client(session, "ACTION", |client| client.action(signal_id))
}

#[tauri::command]
fn save_server_layout(
    runtime: State<'_, AppRuntime>,
    layout_name: String,
    layout_json: String,
) -> Result<(), String> {
    let session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;
    let session = session_slot
        .as_ref()
        .ok_or_else(|| String::from("not connected"))?;
    with_control_client(session, "LAYOUT_SAVE", |client| {
        client.save_layout(layout_name.trim(), &layout_json)
    })
}

#[tauri::command]
fn load_server_layout(
    runtime: State<'_, AppRuntime>,
    layout_name: String,
) -> Result<String, String> {
    let session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;
    let session = session_slot
        .as_ref()
        .ok_or_else(|| String::from("not connected"))?;
    with_control_client(session, "LAYOUT_LOAD", |client| {
        client.load_layout(layout_name.trim())
    })
}

#[tauri::command]
fn list_server_layouts(runtime: State<'_, AppRuntime>) -> Result<Vec<String>, String> {
    let session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;
    let session = session_slot
        .as_ref()
        .ok_or_else(|| String::from("not connected"))?;
    with_control_client(session, "LAYOUT_LIST", |client| client.list_layouts())
}

#[tauri::command]
fn delete_server_layout(runtime: State<'_, AppRuntime>, layout_name: String) -> Result<(), String> {
    let session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;
    let session = session_slot
        .as_ref()
        .ok_or_else(|| String::from("not connected"))?;
    with_control_client(session, "LAYOUT_DELETE", |client| {
        client.delete_layout(layout_name.trim())
    })
}

#[tauri::command]
fn window_mode_snapshot(
    runtime: State<'_, AppRuntime>,
    window: Window,
) -> Result<WindowModeSnapshot, String> {
    let presentation_mode = window
        .is_fullscreen()
        .map_err(|err| format!("read fullscreen state failed: {err}"))?;
    runtime
        .presentation_mode
        .store(presentation_mode, Ordering::SeqCst);

    Ok(WindowModeSnapshot {
        presentation_mode,
        dock_mode: runtime.dock_mode.load(Ordering::SeqCst),
    })
}

#[tauri::command]
fn set_presentation_mode(
    runtime: State<'_, AppRuntime>,
    window: Window,
    enabled: bool,
) -> Result<(), String> {
    window
        .set_fullscreen(enabled)
        .map_err(|err| format!("set fullscreen failed: {err}"))?;

    if enabled {
        runtime.dock_mode.store(false, Ordering::SeqCst);
        window
            .set_always_on_top(false)
            .map_err(|err| format!("disable always-on-top failed: {err}"))?;
    } else if !runtime.dock_mode.load(Ordering::SeqCst) {
        window
            .set_decorations(true)
            .map_err(|err| format!("restore window decorations failed: {err}"))?;
        window
            .set_resizable(true)
            .map_err(|err| format!("restore window resize failed: {err}"))?;
    }

    runtime.presentation_mode.store(enabled, Ordering::SeqCst);
    if enabled {
        let _ = window.set_focus();
    }
    Ok(())
}

fn build_signal_row(state: &DashboardState, descriptor: &ManifestItem) -> SignalRow {
    let value = state
        .value_for(descriptor.signal_id)
        .map(format_signal_value)
        .unwrap_or_else(|| String::from("-"));
    SignalRow {
        signal_id: descriptor.signal_id,
        signal_type: descriptor.signal_type.as_str().to_string(),
        kind: descriptor.kind.as_str().to_string(),
        access: descriptor.access.as_str().to_string(),
        policy: descriptor.policy.as_str().to_string(),
        durability: descriptor.durability.as_str().to_string(),
        path: descriptor.path.clone(),
        value,
    }
}

#[tauri::command]
fn set_driverstation_dock_mode(
    runtime: State<'_, AppRuntime>,
    window: Window,
    enabled: bool,
) -> Result<(), String> {
    if !enabled {
        runtime.dock_mode.store(false, Ordering::SeqCst);
        window
            .set_always_on_top(false)
            .map_err(|err| format!("disable always-on-top failed: {err}"))?;
        window
            .set_decorations(true)
            .map_err(|err| format!("restore window decorations failed: {err}"))?;
        window
            .set_resizable(true)
            .map_err(|err| format!("restore window resize failed: {err}"))?;
        return Ok(());
    }

    window
        .set_fullscreen(false)
        .map_err(|err| format!("exit fullscreen failed: {err}"))?;
    runtime.presentation_mode.store(false, Ordering::SeqCst);

    let monitor = match window
        .current_monitor()
        .map_err(|err| format!("read current monitor failed: {err}"))?
    {
        Some(monitor) => monitor,
        None => window
            .primary_monitor()
            .map_err(|err| format!("read primary monitor failed: {err}"))?
            .ok_or_else(|| String::from("no monitor available"))?,
    };

    let work_area = monitor.work_area();
    let width = work_area.size.width;
    let max_height = work_area.size.height;
    let min_height = max_height.min(260);
    let suggested_height = ((max_height as f64) * 0.37).round() as u32;
    let height = suggested_height.clamp(min_height, max_height);

    window
        .set_decorations(false)
        .map_err(|err| format!("set borderless mode failed: {err}"))?;
    window
        .set_resizable(false)
        .map_err(|err| format!("set non-resizable failed: {err}"))?;
    window
        .set_size(PhysicalSize::new(width, height))
        .map_err(|err| format!("set dock size failed: {err}"))?;
    window
        .set_position(PhysicalPosition::new(
            work_area.position.x,
            work_area.position.y,
        ))
        .map_err(|err| format!("set dock position failed: {err}"))?;
    window
        .set_always_on_top(true)
        .map_err(|err| format!("enable always-on-top failed: {err}"))?;

    let _ = window.set_focus();
    runtime.dock_mode.store(true, Ordering::SeqCst);
    Ok(())
}

fn with_control_client<T>(
    session: &Session,
    context: &str,
    f: impl FnOnce(&mut ControlClient) -> io::Result<T>,
) -> Result<T, String> {
    let mut guard = session
        .control
        .lock()
        .map_err(|_| String::from("control lock poisoned"))?;
    let client = guard
        .client
        .as_mut()
        .ok_or_else(|| String::from("control disconnected; waiting for reconnect"))?;
    match f(client) {
        Ok(v) => Ok(v),
        Err(err) => {
            guard.client = None;
            guard.connected = false;
            guard.status = format!("{context} failed: {err}; reconnecting...");
            Err(guard.status.clone())
        }
    }
}

fn spawn_control_worker(
    running: Arc<AtomicBool>,
    state: Arc<Mutex<DashboardState>>,
    control: Arc<Mutex<ControlRuntime>>,
    host: String,
    control_port: u16,
    udp_port: u16,
) -> io::Result<JoinHandle<()>> {
    thread::Builder::new()
        .name("arcp-dashboard-control".to_string())
        .spawn(move || {
            let mut next_healthcheck = Instant::now() + CONTROL_HEALTHCHECK_INTERVAL;
            let mut next_reconnect = Instant::now() + CONTROL_RECONNECT_INTERVAL;

            while running.load(Ordering::SeqCst) {
                let now = Instant::now();
                let mut needs_reconnect = false;

                if let Ok(mut guard) = control.lock() {
                    if let Some(client) = guard.client.as_mut() {
                        if now >= next_healthcheck {
                            next_healthcheck = now + CONTROL_HEALTHCHECK_INTERVAL;
                            if let Err(err) = client.ping() {
                                guard.client = None;
                                guard.connected = false;
                                guard.status = format!("control link lost: {err}; reconnecting...");
                                needs_reconnect = true;
                                next_reconnect = now;
                            }
                        }
                    } else {
                        needs_reconnect = true;
                    }
                }

                if needs_reconnect && now >= next_reconnect {
                    next_reconnect = now + CONTROL_RECONNECT_INTERVAL;

                    match reconnect_control(&host, control_port, udp_port, &state) {
                        Ok(client) => {
                            if let Ok(mut guard) = control.lock() {
                                guard.client = Some(client);
                                guard.connected = true;
                                guard.status = format!(
                                    "reconnected to {}:{} (udp {})",
                                    host, control_port, udp_port
                                );
                            }
                            next_healthcheck = Instant::now() + CONTROL_HEALTHCHECK_INTERVAL;
                        }
                        Err(err) => {
                            if let Ok(mut guard) = control.lock() {
                                guard.connected = false;
                                guard.status = format!("reconnect failed: {err}");
                            }
                        }
                    }
                }

                thread::sleep(Duration::from_millis(200));
            }

            if let Ok(mut guard) = control.lock() {
                if let Some(client) = guard.client.as_mut() {
                    let _ = client.unsubscribe(udp_port);
                }
                guard.client = None;
                guard.connected = false;
                guard.status = String::from("disconnected");
            }
        })
}

fn reconnect_control(
    host: &str,
    control_port: u16,
    udp_port: u16,
    state: &Arc<Mutex<DashboardState>>,
) -> io::Result<ControlClient> {
    let mut client = ControlClient::connect(host, control_port)?;
    client.ping()?;
    let manifest = client.manifest()?;
    client.subscribe(udp_port)?;
    if let Ok(mut guard) = state.lock() {
        guard.replace_descriptors(manifest);
    }
    Ok(client)
}

#[cfg(target_os = "linux")]
fn read_rss_bytes() -> Option<u64> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    for line in status.lines() {
        if let Some(rest) = line.strip_prefix("VmRSS:") {
            let kb = rest.split_whitespace().next()?.parse::<u64>().ok()?;
            return Some(kb.saturating_mul(1024));
        }
    }
    None
}

#[cfg(not(target_os = "linux"))]
fn read_rss_bytes() -> Option<u64> {
    None
}

#[cfg(target_os = "linux")]
fn read_process_jiffies() -> Option<u64> {
    let stat = std::fs::read_to_string("/proc/self/stat").ok()?;
    let rparen = stat.rfind(')')?;
    let rest = stat.get(rparen + 2..)?;
    let mut fields = rest.split_whitespace();
    for _ in 0..11 {
        fields.next()?;
    }
    let utime = fields.next()?.parse::<u64>().ok()?;
    let stime = fields.next()?.parse::<u64>().ok()?;
    Some(utime.saturating_add(stime))
}

#[cfg(not(target_os = "linux"))]
fn read_process_jiffies() -> Option<u64> {
    None
}

#[cfg(target_os = "linux")]
fn read_total_jiffies() -> Option<u64> {
    let stat = std::fs::read_to_string("/proc/stat").ok()?;
    let line = stat.lines().next()?;
    let mut fields = line.split_whitespace();
    if fields.next()? != "cpu" {
        return None;
    }
    let mut total = 0_u64;
    for raw in fields {
        total = total.saturating_add(raw.parse::<u64>().ok()?);
    }
    Some(total)
}

#[cfg(not(target_os = "linux"))]
fn read_total_jiffies() -> Option<u64> {
    None
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .manage(AppRuntime::default())
        .invoke_handler(tauri::generate_handler![
            connect_arcp,
            disconnect_arcp,
            dashboard_snapshot,
            dashboard_delta,
            set_signal_value,
            trigger_action,
            save_server_layout,
            load_server_layout,
            list_server_layouts,
            delete_server_layout,
            window_mode_snapshot,
            set_presentation_mode,
            set_driverstation_dock_mode,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
