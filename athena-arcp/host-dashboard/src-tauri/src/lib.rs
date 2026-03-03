use std::io;
use std::net::{TcpStream, ToSocketAddrs};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use arcp_dashboard::{
    format_signal_value, ControlClient, DashboardState, ManifestItem, TelemetrySubscription,
};
use serde::Serialize;
use tauri::{Emitter, PhysicalPosition, PhysicalSize, State, Window};
#[cfg(target_os = "windows")]
use windows_sys::Win32::Foundation::{BOOL, HWND, LPARAM, RECT};
#[cfg(target_os = "windows")]
use windows_sys::Win32::Graphics::Gdi::{
    GetMonitorInfoW, MonitorFromWindow, MONITORINFO, MONITOR_DEFAULTTONEAREST,
};
#[cfg(target_os = "windows")]
use windows_sys::Win32::UI::WindowsAndMessaging::{
    EnumWindows, GetWindowRect, GetWindowTextLengthW, GetWindowTextW, IsIconic, IsWindowVisible,
};

const CONTROL_RECONNECT_INTERVAL: Duration = Duration::from_millis(1000);
const CONTROL_HEALTHCHECK_INTERVAL: Duration = Duration::from_millis(1500);
const SERVER_STATS_REFRESH_INTERVAL: Duration = Duration::from_millis(500);
const RIO_TCP_PROBE_TIMEOUT: Duration = Duration::from_millis(180);
const RIO_TCP_PROBE_PORTS: [u16; 2] = [22, 80];
const ARCP_TCP_PROBE_TIMEOUT: Duration = Duration::from_millis(220);
const DRIVERSTATION_HOST_ALIASES: [&str; 4] = ["ds", "driverstation", "driver-station", "auto"];
#[cfg(target_os = "windows")]
const DRIVERSTATION_STORAGE_PATH: &str = r"C:\Users\Public\Documents\FRC\FRC DS Data Storage.ini";
const SSH_REMOTE_USERS: [&str; 2] = ["lvuser", "admin"];
const REMOTE_LOG_PREVIEW_BYTES_DEFAULT: usize = 64 * 1024;
const REMOTE_LOG_PREVIEW_BYTES_MAX: usize = 512 * 1024;
const REMOTE_LOG_DOWNLOAD_BYTES_LIMIT_DEFAULT: u64 = 20 * 1024 * 1024;
const REMOTE_LOG_DOWNLOAD_BYTES_LIMIT_MAX: u64 = 64 * 1024 * 1024;
const REMOTE_LOG_LIST_SCRIPT: &str = r#"{ for dir in /home/lvuser/athena/logs /home/lvuser/logs /u/logs /var/log; do if [ -d "$dir" ]; then find "$dir" -maxdepth 5 -type f 2>/dev/null; fi; done; if [ -f /home/lvuser/FRC_UserProgram.log ]; then echo /home/lvuser/FRC_UserProgram.log; fi; } | while IFS= read -r f; do case "$f" in *.log|*.txt|*.json|*.wpilog|*.gz) size=$(stat -c %s "$f" 2>/dev/null || wc -c < "$f" 2>/dev/null || echo 0); mtime=$(stat -c %Y "$f" 2>/dev/null || date -r "$f" +%s 2>/dev/null || echo 0); printf '%s|%s|%s\n' "$mtime" "$size" "$f";; esac; done | sort -t'|' -k1,1nr | head -n 800"#;
const REMOTE_LOG_ALLOWED_PREFIXES: [&str; 4] = [
    "/home/lvuser/athena/logs/",
    "/home/lvuser/logs/",
    "/u/logs/",
    "/var/log/",
];
const REMOTE_LOG_ALLOWED_FILES: [&str; 1] = ["/home/lvuser/FRC_UserProgram.log"];

struct AppRuntime {
    session: Mutex<Option<Session>>,
    remote_log_stream: Mutex<Option<RemoteLogStreamSession>>,
    presentation_mode: AtomicBool,
    dock_mode: AtomicBool,
}

impl Default for AppRuntime {
    fn default() -> Self {
        Self {
            session: Mutex::new(None),
            remote_log_stream: Mutex::new(None),
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

struct RemoteLogStreamSession {
    running: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl RemoteLogStreamSession {
    fn stop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
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

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct RemoteLogEntry {
    source: String,
    path: String,
    name: String,
    size_bytes: u64,
    modified_epoch_sec: u64,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct RemoteLogList {
    host: String,
    entries: Vec<RemoteLogEntry>,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct RemoteLogPreview {
    host: String,
    path: String,
    content: String,
    truncated: bool,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct RemoteLogDownload {
    host: String,
    path: String,
    file_name: String,
    bytes: Vec<u8>,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct RobotLinkProbe {
    host: String,
    control_port: u16,
    robot_reachable: bool,
    arcp_reachable: bool,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct RemoteLogStreamLineEvent {
    host: String,
    path: String,
    line: String,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct RemoteLogStreamStatusEvent {
    host: String,
    path: String,
    state: String,
    message: String,
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

fn parse_team_number(raw: &str) -> Option<u16> {
    let trimmed = raw.trim().trim_matches('"');
    if trimmed.is_empty() || !trimmed.chars().all(|ch| ch.is_ascii_digit()) {
        return None;
    }

    let team = trimmed.parse::<u16>().ok()?;
    if !(1..=25_599).contains(&team) {
        return None;
    }
    Some(team)
}

fn team_number_to_roborio_ip(team: u16) -> String {
    let major = team / 100;
    let minor = team % 100;
    format!("10.{major}.{minor}.2")
}

fn roborio_candidates_for_team(team: u16) -> Vec<String> {
    vec![
        team_number_to_roborio_ip(team),
        String::from("172.22.11.2"),
        format!("roborio-{team}-frc.local"),
    ]
}

fn probe_host_port(host: &str, port: u16) -> bool {
    let Ok(addrs) = (host, port).to_socket_addrs() else {
        return false;
    };

    for addr in addrs {
        match TcpStream::connect_timeout(&addr, RIO_TCP_PROBE_TIMEOUT) {
            Ok(_) => return true,
            Err(err)
                if matches!(
                    err.kind(),
                    io::ErrorKind::ConnectionRefused
                        | io::ErrorKind::ConnectionReset
                        | io::ErrorKind::PermissionDenied
                ) =>
            {
                // A refusal/reset still proves the target host is reachable.
                return true;
            }
            Err(_) => {}
        }
    }

    false
}

fn is_roborio_host_reachable(host: &str) -> bool {
    RIO_TCP_PROBE_PORTS
        .iter()
        .copied()
        .any(|port| probe_host_port(host, port))
}

fn is_tcp_port_open(host: &str, port: u16, timeout: Duration) -> bool {
    let Ok(addrs) = (host, port).to_socket_addrs() else {
        return false;
    };

    for addr in addrs {
        if TcpStream::connect_timeout(&addr, timeout).is_ok() {
            return true;
        }
    }

    false
}

fn select_roborio_host_for_team(team: u16) -> String {
    let candidates = roborio_candidates_for_team(team);
    for candidate in &candidates {
        if is_roborio_host_reachable(candidate) {
            return candidate.clone();
        }
    }
    candidates
        .into_iter()
        .next()
        .unwrap_or_else(|| team_number_to_roborio_ip(team))
}

fn parse_team_alias(raw: &str) -> Option<u16> {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return None;
    }

    if let Some(team) = parse_team_number(trimmed) {
        return Some(team);
    }

    let lowered = trimmed.to_ascii_lowercase();
    let rest = lowered
        .strip_prefix("team:")
        .or_else(|| lowered.strip_prefix("team="))
        .or_else(|| lowered.strip_prefix("frc"))?;

    parse_team_number(rest)
}

fn should_resolve_driverstation_host(raw: &str) -> bool {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return true;
    }

    let lowered = trimmed.to_ascii_lowercase();
    DRIVERSTATION_HOST_ALIASES
        .iter()
        .any(|alias| lowered == *alias)
}

#[cfg(target_os = "windows")]
fn read_ini_value(contents: &str, section: &str, key: &str) -> Option<String> {
    let mut in_section = false;

    for line in contents.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with(';') || trimmed.starts_with('#') {
            continue;
        }

        if let Some(inner) = trimmed
            .strip_prefix('[')
            .and_then(|value| value.strip_suffix(']'))
        {
            in_section = inner.trim().eq_ignore_ascii_case(section);
            continue;
        }

        if !in_section {
            continue;
        }

        let Some((lhs, rhs)) = trimmed.split_once('=') else {
            continue;
        };

        if !lhs.trim().eq_ignore_ascii_case(key) {
            continue;
        }

        let mut value = rhs.trim().to_string();
        if value.starts_with('"') && value.ends_with('"') && value.len() >= 2 {
            value = value[1..value.len() - 1].to_string();
        }
        return Some(value);
    }

    None
}

#[cfg(target_os = "windows")]
fn read_driverstation_team_number() -> Result<u16, String> {
    let contents = std::fs::read_to_string(DRIVERSTATION_STORAGE_PATH).map_err(|err| {
        format!("Driver Station config read failed ({DRIVERSTATION_STORAGE_PATH}): {err}")
    })?;

    let raw_team = read_ini_value(&contents, "Setup", "TeamNumber")
        .ok_or_else(|| String::from("Driver Station team number is missing"))?;
    parse_team_number(&raw_team)
        .ok_or_else(|| format!("Driver Station team number is invalid: {raw_team}"))
}

fn resolve_control_host(requested_host: &str) -> Result<String, String> {
    let trimmed = requested_host.trim();

    if let Some(team) = parse_team_alias(trimmed) {
        return Ok(select_roborio_host_for_team(team));
    }

    if !should_resolve_driverstation_host(trimmed) {
        return Ok(trimmed.to_string());
    }

    #[cfg(target_os = "windows")]
    {
        let team = read_driverstation_team_number()?;
        return Ok(select_roborio_host_for_team(team));
    }

    #[cfg(not(target_os = "windows"))]
    {
        if trimmed.is_empty() {
            return Err(String::from(
                "host must not be empty (Driver Station auto host is Windows-only)",
            ));
        }
        return Err(String::from(
            "Driver Station host resolution is only available on Windows desktop",
        ));
    }
}

fn resolve_remote_target_host(
    runtime: &AppRuntime,
    requested_host: &str,
) -> Result<String, String> {
    let trimmed = requested_host.trim();
    if should_resolve_driverstation_host(trimmed) {
        if let Ok(session_slot) = runtime.session.lock() {
            if let Some(session) = session_slot.as_ref() {
                return Ok(session.host.clone());
            }
        }
    }
    resolve_control_host(trimmed)
}

fn run_ssh_command(host: &str, remote_command: &str) -> Result<Vec<u8>, String> {
    let mut last_error = String::new();

    for user in SSH_REMOTE_USERS {
        let destination = format!("{user}@{host}");
        let output = Command::new("ssh")
            .arg("-o")
            .arg("BatchMode=yes")
            .arg("-o")
            .arg("ConnectTimeout=3")
            .arg("-o")
            .arg("StrictHostKeyChecking=no")
            .arg(destination.as_str())
            .arg(remote_command)
            .output();

        match output {
            Ok(result) if result.status.success() => return Ok(result.stdout),
            Ok(result) => {
                let stderr = String::from_utf8_lossy(&result.stderr).trim().to_string();
                let stdout = String::from_utf8_lossy(&result.stdout).trim().to_string();
                last_error = if !stderr.is_empty() {
                    format!("{destination}: {stderr}")
                } else if !stdout.is_empty() {
                    format!("{destination}: {stdout}")
                } else {
                    format!("{destination}: ssh exited with status {}", result.status)
                };
            }
            Err(err) if err.kind() == io::ErrorKind::NotFound => {
                return Err(String::from(
                    "OpenSSH client not found on host machine (expected `ssh` in PATH)",
                ));
            }
            Err(err) => {
                last_error = format!("{destination}: failed to run ssh: {err}");
            }
        }
    }

    Err(format!("remote ssh command failed: {last_error}"))
}

fn classify_remote_log_source(path: &str) -> &'static str {
    if path.contains("/athena/") {
        "athena"
    } else {
        "rio"
    }
}

fn is_allowed_remote_log_path(path: &str) -> bool {
    REMOTE_LOG_ALLOWED_FILES.iter().any(|entry| path == *entry)
        || REMOTE_LOG_ALLOWED_PREFIXES
            .iter()
            .any(|prefix| path.starts_with(prefix))
}

fn shell_quote_single(raw: &str) -> String {
    format!("'{}'", raw.replace('\'', "'\"'\"'"))
}

fn remote_log_file_size_bytes(host: &str, path: &str) -> Result<u64, String> {
    let quoted = shell_quote_single(path);
    let command = format!("wc -c < {quoted}");
    let output = run_ssh_command(host, &command)?;
    let raw = String::from_utf8_lossy(&output).trim().to_string();
    raw.parse::<u64>()
        .map_err(|_| format!("failed to parse remote file size for {path}: {raw}"))
}

fn emit_remote_log_stream_status(window: &Window, host: &str, path: &str, state: &str, message: &str) {
    let _ = window.emit(
        "remote-log-stream-status",
        RemoteLogStreamStatusEvent {
            host: host.to_string(),
            path: path.to_string(),
            state: state.to_string(),
            message: message.to_string(),
        },
    );
}

fn spawn_remote_log_stream_worker(
    window: Window,
    running: Arc<AtomicBool>,
    host: String,
    path: String,
) -> JoinHandle<()> {
    thread::spawn(move || {
        let quoted_path = shell_quote_single(&path);
        let remote_command = format!("tail -n 200 -F -- {quoted_path}");

        emit_remote_log_stream_status(
            &window,
            &host,
            &path,
            "starting",
            "starting remote log stream",
        );

        while running.load(Ordering::SeqCst) {
            let mut attempted_user = false;
            let mut connected = false;

            for user in SSH_REMOTE_USERS {
                if !running.load(Ordering::SeqCst) {
                    break;
                }
                attempted_user = true;
                let destination = format!("{user}@{host}");
                let child = Command::new("ssh")
                    .arg("-o")
                    .arg("BatchMode=yes")
                    .arg("-o")
                    .arg("ConnectTimeout=3")
                    .arg("-o")
                    .arg("StrictHostKeyChecking=no")
                    .arg("-o")
                    .arg("LogLevel=ERROR")
                    .arg(destination.as_str())
                    .arg(remote_command.as_str())
                    .stdout(Stdio::piped())
                    .stderr(Stdio::null())
                    .spawn();

                let mut child = match child {
                    Ok(child) => child,
                    Err(err) if err.kind() == io::ErrorKind::NotFound => {
                        emit_remote_log_stream_status(
                            &window,
                            &host,
                            &path,
                            "error",
                            "OpenSSH client not found on host machine",
                        );
                        return;
                    }
                    Err(err) => {
                        emit_remote_log_stream_status(
                            &window,
                            &host,
                            &path,
                            "error",
                            &format!("stream launch failed for {destination}: {err}"),
                        );
                        continue;
                    }
                };

                connected = true;
                emit_remote_log_stream_status(
                    &window,
                    &host,
                    &path,
                    "connected",
                    &format!("stream connected via {destination}"),
                );

                if let Some(stdout) = child.stdout.take() {
                    let reader = io::BufReader::new(stdout);
                    for raw_line in io::BufRead::lines(reader) {
                        if !running.load(Ordering::SeqCst) {
                            break;
                        }
                        match raw_line {
                            Ok(line) => {
                                let _ = window.emit(
                                    "remote-log-stream-line",
                                    RemoteLogStreamLineEvent {
                                        host: host.clone(),
                                        path: path.clone(),
                                        line,
                                    },
                                );
                            }
                            Err(err) => {
                                emit_remote_log_stream_status(
                                    &window,
                                    &host,
                                    &path,
                                    "error",
                                    &format!("stream read failed: {err}"),
                                );
                                break;
                            }
                        }
                    }
                }

                let _ = child.kill();
                let _ = child.wait();
                if !running.load(Ordering::SeqCst) {
                    break;
                }
            }

            if !running.load(Ordering::SeqCst) {
                break;
            }

            if !attempted_user {
                emit_remote_log_stream_status(
                    &window,
                    &host,
                    &path,
                    "error",
                    "no SSH user candidates configured",
                );
                break;
            }

            let reconnect_message = if connected {
                "log stream disconnected; reconnecting"
            } else {
                "unable to establish remote log stream; retrying"
            };
            emit_remote_log_stream_status(&window, &host, &path, "reconnecting", reconnect_message);
            thread::sleep(Duration::from_millis(700));
        }

        emit_remote_log_stream_status(&window, &host, &path, "stopped", "log stream stopped");
    })
}

fn stop_remote_log_stream_internal(runtime: &AppRuntime) -> Result<(), String> {
    let mut stream_slot = runtime
        .remote_log_stream
        .lock()
        .map_err(|_| String::from("remote log stream lock poisoned"))?;
    if let Some(mut stream) = stream_slot.take() {
        stream.stop();
    }
    Ok(())
}

#[tauri::command]
fn list_remote_logs(runtime: State<'_, AppRuntime>, host: String) -> Result<RemoteLogList, String> {
    let resolved_host = resolve_remote_target_host(&runtime, &host)?;
    let output = run_ssh_command(&resolved_host, REMOTE_LOG_LIST_SCRIPT)?;
    let mut entries = Vec::new();
    let mut seen = std::collections::HashSet::new();

    for line in String::from_utf8_lossy(&output).lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        let mut parts = trimmed.splitn(3, '|');
        let modified_epoch_sec = parts
            .next()
            .and_then(|raw| raw.parse::<u64>().ok())
            .unwrap_or(0);
        let size_bytes = parts
            .next()
            .and_then(|raw| raw.parse::<u64>().ok())
            .unwrap_or(0);
        let Some(path) = parts.next().map(str::trim) else {
            continue;
        };
        if path.is_empty() || !is_allowed_remote_log_path(path) || !seen.insert(path.to_string()) {
            continue;
        }

        let name = path.rsplit('/').next().unwrap_or(path).to_string();
        entries.push(RemoteLogEntry {
            source: String::from(classify_remote_log_source(path)),
            path: path.to_string(),
            name,
            size_bytes,
            modified_epoch_sec,
        });
    }

    Ok(RemoteLogList {
        host: resolved_host,
        entries,
    })
}

#[tauri::command]
fn read_remote_log_preview(
    runtime: State<'_, AppRuntime>,
    host: String,
    path: String,
    max_bytes: Option<u32>,
) -> Result<RemoteLogPreview, String> {
    let resolved_host = resolve_remote_target_host(&runtime, &host)?;
    let normalized_path = path.trim();
    if normalized_path.is_empty() {
        return Err(String::from("remote log path must not be empty"));
    }
    if !is_allowed_remote_log_path(normalized_path) {
        return Err(String::from(
            "remote log path is outside allowed log directories",
        ));
    }

    let limit = max_bytes
        .map(|value| value as usize)
        .unwrap_or(REMOTE_LOG_PREVIEW_BYTES_DEFAULT)
        .clamp(1024, REMOTE_LOG_PREVIEW_BYTES_MAX);
    let size_bytes = remote_log_file_size_bytes(&resolved_host, normalized_path)?;
    let quoted = shell_quote_single(normalized_path);
    let command = format!("tail -c {limit} -- {quoted} 2>/dev/null || cat -- {quoted} 2>/dev/null");
    let output = run_ssh_command(&resolved_host, &command)?;

    Ok(RemoteLogPreview {
        host: resolved_host,
        path: normalized_path.to_string(),
        content: String::from_utf8_lossy(&output).to_string(),
        truncated: size_bytes > (limit as u64),
    })
}

#[tauri::command]
fn download_remote_log(
    runtime: State<'_, AppRuntime>,
    host: String,
    path: String,
    max_bytes: Option<u64>,
) -> Result<RemoteLogDownload, String> {
    let resolved_host = resolve_remote_target_host(&runtime, &host)?;
    let normalized_path = path.trim();
    if normalized_path.is_empty() {
        return Err(String::from("remote log path must not be empty"));
    }
    if !is_allowed_remote_log_path(normalized_path) {
        return Err(String::from(
            "remote log path is outside allowed log directories",
        ));
    }

    let limit = max_bytes
        .unwrap_or(REMOTE_LOG_DOWNLOAD_BYTES_LIMIT_DEFAULT)
        .clamp(1024, REMOTE_LOG_DOWNLOAD_BYTES_LIMIT_MAX);
    let size_bytes = remote_log_file_size_bytes(&resolved_host, normalized_path)?;
    if size_bytes > limit {
        return Err(format!(
            "remote file is {size_bytes} bytes, exceeds current download limit of {limit} bytes"
        ));
    }

    let quoted = shell_quote_single(normalized_path);
    let output = run_ssh_command(&resolved_host, &format!("cat -- {quoted}"))?;
    let file_name = normalized_path
        .rsplit('/')
        .next()
        .unwrap_or(normalized_path)
        .to_string();

    Ok(RemoteLogDownload {
        host: resolved_host,
        path: normalized_path.to_string(),
        file_name,
        bytes: output,
    })
}

#[tauri::command]
fn probe_robot_link(
    runtime: State<'_, AppRuntime>,
    host: String,
    control_port: u16,
) -> Result<RobotLinkProbe, String> {
    let resolved_host = resolve_remote_target_host(&runtime, &host)?;
    let robot_reachable = is_roborio_host_reachable(&resolved_host);
    let arcp_reachable = is_tcp_port_open(&resolved_host, control_port, ARCP_TCP_PROBE_TIMEOUT);

    Ok(RobotLinkProbe {
        host: resolved_host,
        control_port,
        robot_reachable,
        arcp_reachable,
    })
}

#[tauri::command]
fn connect_arcp(
    runtime: State<'_, AppRuntime>,
    host: String,
    control_port: u16,
) -> Result<ConnectInfo, String> {
    let host = resolve_control_host(&host)?;

    let mut session_slot = runtime
        .session
        .lock()
        .map_err(|_| String::from("runtime lock poisoned"))?;

    if let Some(mut existing) = session_slot.take() {
        existing.stop();
    }

    let mut control = ControlClient::connect(&host, control_port)
        .map_err(|err| format!("connect failed ({host}:{control_port}): {err}"))?;
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

fn default_dock_geometry(
    window: &Window,
) -> Result<(PhysicalPosition<i32>, PhysicalSize<u32>), String> {
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

    Ok((
        PhysicalPosition::new(work_area.position.x, work_area.position.y),
        PhysicalSize::new(width, height),
    ))
}

#[cfg(target_os = "windows")]
#[derive(Clone, Copy)]
struct DriverStationWindowMatch {
    hwnd: HWND,
    rect: RECT,
    area: i64,
}

#[cfg(target_os = "windows")]
#[derive(Clone, Copy)]
struct WindowRect {
    left: i32,
    top: i32,
    right: i32,
    bottom: i32,
}

#[cfg(target_os = "windows")]
impl WindowRect {
    fn from_rect(rect: RECT) -> Self {
        Self {
            left: rect.left,
            top: rect.top,
            right: rect.right,
            bottom: rect.bottom,
        }
    }

    fn width(self) -> i32 {
        (self.right - self.left).max(0)
    }

    fn height(self) -> i32 {
        (self.bottom - self.top).max(0)
    }

    fn area(self) -> i64 {
        (self.width() as i64).saturating_mul(self.height() as i64)
    }
}

#[cfg(target_os = "windows")]
fn normalize_window_title(raw: &str) -> String {
    raw.chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .collect::<String>()
        .to_ascii_lowercase()
}

#[cfg(target_os = "windows")]
fn find_driverstation_window() -> Option<(HWND, RECT)> {
    unsafe extern "system" fn enum_windows_proc(hwnd: HWND, lparam: LPARAM) -> BOOL {
        if lparam == 0 {
            return 1;
        }
        if unsafe { IsWindowVisible(hwnd) } == 0 || unsafe { IsIconic(hwnd) } != 0 {
            return 1;
        }

        let title_len = unsafe { GetWindowTextLengthW(hwnd) };
        if title_len <= 0 {
            return 1;
        }

        let mut title_buf = vec![0_u16; (title_len as usize) + 1];
        let written =
            unsafe { GetWindowTextW(hwnd, title_buf.as_mut_ptr(), title_buf.len() as i32) };
        if written <= 0 {
            return 1;
        }

        let title = String::from_utf16_lossy(&title_buf[..written as usize]);
        let normalized = normalize_window_title(&title);
        if !normalized.contains("driverstation") {
            return 1;
        }

        let mut rect: RECT = unsafe { std::mem::zeroed() };
        if unsafe { GetWindowRect(hwnd, &mut rect) } == 0 {
            return 1;
        }
        let width = (rect.right - rect.left).max(0) as i64;
        let height = (rect.bottom - rect.top).max(0) as i64;
        let area = width.saturating_mul(height);
        if area == 0 {
            return 1;
        }

        let best_match = unsafe { &mut *(lparam as *mut Option<DriverStationWindowMatch>) };
        let replace = best_match
            .as_ref()
            .map(|current| area > current.area)
            .unwrap_or(true);
        if replace {
            *best_match = Some(DriverStationWindowMatch { hwnd, rect, area });
        }

        1
    }

    let mut best_match: Option<DriverStationWindowMatch> = None;
    unsafe {
        EnumWindows(
            Some(enum_windows_proc),
            (&mut best_match as *mut Option<DriverStationWindowMatch>) as LPARAM,
        );
    }
    best_match.map(|entry| (entry.hwnd, entry.rect))
}

#[cfg(target_os = "windows")]
fn monitor_work_area_for_window(hwnd: HWND) -> Option<RECT> {
    let monitor = unsafe { MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST) };
    if monitor.is_null() {
        return None;
    }

    let mut info: MONITORINFO = unsafe { std::mem::zeroed() };
    info.cbSize = std::mem::size_of::<MONITORINFO>() as u32;
    let ok = unsafe { GetMonitorInfoW(monitor, &mut info as *mut MONITORINFO as *mut _) };
    if ok == 0 {
        return None;
    }
    Some(info.rcWork)
}

#[cfg(target_os = "windows")]
fn clamp_rect(rect: WindowRect, bounds: WindowRect) -> WindowRect {
    let mut left = rect.left.clamp(bounds.left, bounds.right);
    let mut right = rect.right.clamp(bounds.left, bounds.right);
    if right < left {
        std::mem::swap(&mut left, &mut right);
    }

    let mut top = rect.top.clamp(bounds.top, bounds.bottom);
    let mut bottom = rect.bottom.clamp(bounds.top, bounds.bottom);
    if bottom < top {
        std::mem::swap(&mut top, &mut bottom);
    }

    WindowRect {
        left,
        top,
        right,
        bottom,
    }
}

#[cfg(target_os = "windows")]
fn resolve_windows_driverstation_dock_geometry(
) -> Option<(PhysicalPosition<i32>, PhysicalSize<u32>)> {
    let (driverstation_hwnd, driverstation_rect_raw) = find_driverstation_window()?;
    let work_area_raw = monitor_work_area_for_window(driverstation_hwnd)?;

    let work_area = WindowRect::from_rect(work_area_raw);
    if work_area.width() <= 0 || work_area.height() <= 0 {
        return None;
    }

    let driverstation_rect = clamp_rect(WindowRect::from_rect(driverstation_rect_raw), work_area);
    if driverstation_rect.width() <= 0 || driverstation_rect.height() <= 0 {
        return None;
    }

    let candidates = [
        WindowRect {
            left: work_area.left,
            top: work_area.top,
            right: work_area.right,
            bottom: driverstation_rect.top,
        },
        WindowRect {
            left: work_area.left,
            top: driverstation_rect.bottom,
            right: work_area.right,
            bottom: work_area.bottom,
        },
        WindowRect {
            left: work_area.left,
            top: work_area.top,
            right: driverstation_rect.left,
            bottom: work_area.bottom,
        },
        WindowRect {
            left: driverstation_rect.right,
            top: work_area.top,
            right: work_area.right,
            bottom: work_area.bottom,
        },
    ];

    let best = candidates
        .iter()
        .copied()
        .filter(|rect| rect.width() > 0 && rect.height() > 0)
        .max_by_key(|rect| rect.area())?;

    if best.width() < 320 || best.height() < 180 {
        return None;
    }

    Some((
        PhysicalPosition::new(best.left, best.top),
        PhysicalSize::new(best.width() as u32, best.height() as u32),
    ))
}

#[cfg(target_os = "windows")]
fn resolve_driverstation_dock_geometry(
    window: &Window,
) -> Result<(PhysicalPosition<i32>, PhysicalSize<u32>), String> {
    if let Some(geometry) = resolve_windows_driverstation_dock_geometry() {
        return Ok(geometry);
    }
    default_dock_geometry(window)
}

#[cfg(not(target_os = "windows"))]
fn resolve_driverstation_dock_geometry(
    window: &Window,
) -> Result<(PhysicalPosition<i32>, PhysicalSize<u32>), String> {
    default_dock_geometry(window)
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

    let (dock_position, dock_size) = resolve_driverstation_dock_geometry(&window)?;

    window
        .set_decorations(false)
        .map_err(|err| format!("set borderless mode failed: {err}"))?;
    window
        .set_resizable(false)
        .map_err(|err| format!("set non-resizable failed: {err}"))?;
    window
        .set_size(dock_size)
        .map_err(|err| format!("set dock size failed: {err}"))?;
    window
        .set_position(dock_position)
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
            list_remote_logs,
            read_remote_log_preview,
            download_remote_log,
            probe_robot_link,
            window_mode_snapshot,
            set_presentation_mode,
            set_driverstation_dock_mode,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
