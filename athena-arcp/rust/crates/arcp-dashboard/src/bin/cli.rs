use std::io::{self, Stdout};
use std::panic::{self, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use arcp_dashboard::{
    format_signal_value, parse_cli_args, CliArgs, ControlClient, DashboardState,
    TelemetrySubscription,
};
use crossterm::cursor::{Hide, Show};
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use crossterm::execute;
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Modifier, Style};
use ratatui::widgets::{Block, Borders, Clear, Paragraph, Row, Table};
use ratatui::{Frame, Terminal};

const KEY_POLL_INTERVAL: Duration = Duration::from_millis(20);
const STATS_REFRESH_INTERVAL: Duration = Duration::from_millis(500);
const CONTROL_RECONNECT_INTERVAL: Duration = Duration::from_millis(1000);
const CONTROL_HEALTHCHECK_INTERVAL: Duration = Duration::from_millis(1500);

fn main() {
    let args = match parse_cli_args(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(msg) => {
            eprintln!("{msg}");
            std::process::exit(2);
        }
    };

    if let Err(err) = run(args) {
        eprintln!("dashboard error: {err}");
        std::process::exit(1);
    }
}

fn run(args: CliArgs) -> io::Result<()> {
    let mut control = ControlClient::connect(&args.host, args.control_port)?;
    control.ping()?;
    let manifest = control.manifest()?;

    let telemetry = TelemetrySubscription::bind_any()?;
    let udp_port = telemetry.local_port()?;
    control.subscribe(udp_port)?;

    let state = Arc::new(Mutex::new(DashboardState::new(manifest)));
    let running = Arc::new(AtomicBool::new(true));
    let telemetry_handle = telemetry.spawn(Arc::clone(&state), Arc::clone(&running))?;

    let ui_result = run_tui_loop(control, &state, &args, args.refresh_ms, udp_port);

    running.store(false, Ordering::SeqCst);
    let _ = telemetry_handle.join();

    ui_result
}

fn run_tui_loop(
    control: ControlClient,
    state: &Arc<Mutex<DashboardState>>,
    args: &CliArgs,
    refresh_ms: u64,
    udp_port: u16,
) -> io::Result<()> {
    let mut session = TerminalSession::enter()?;
    let mut control_link =
        ControlLink::new(control, args.host.clone(), args.control_port, udp_port);
    let mut ui = UiState {
        input: String::new(),
        status: format!(
            "connected to {}:{} (udp subscriber {})",
            args.host, args.control_port, udp_port
        ),
        show_help: false,
        selected_index: 0,
        scroll: 0,
    };

    let mut stats_sampler = StatsSampler::new();
    let refresh_interval = Duration::from_millis(refresh_ms);
    let mut last_render = Instant::now() - refresh_interval;
    let mut last_revision = 0_u64;

    let loop_result = panic::catch_unwind(AssertUnwindSafe(|| -> io::Result<()> {
        loop {
            stats_sampler.maybe_refresh();
            if let Some(reconnect_status) = control_link.maybe_reconnect(state) {
                ui.status = reconnect_status;
            }
            let revision = state.lock().map(|guard| guard.revision()).unwrap_or(0);

            if revision != last_revision || last_render.elapsed() >= refresh_interval {
                let stats = stats_sampler.snapshot();
                session.terminal_mut().draw(|frame| {
                    draw_ui(frame, state, &stats, &mut ui, control_link.connected())
                })?;
                last_revision = revision;
                last_render = Instant::now();
            }

            if event::poll(KEY_POLL_INTERVAL)? {
                match event::read()? {
                    Event::Key(key) if key.kind == KeyEventKind::Press => {
                        if handle_key(key, &mut ui, &mut control_link, state)? {
                            break;
                        }
                    }
                    Event::Resize(_, _) => {
                        last_render = Instant::now() - refresh_interval;
                    }
                    _ => {}
                }
            }
        }

        Ok(())
    }));

    control_link.unsubscribe_best_effort();

    let mut result = match loop_result {
        Ok(inner) => inner,
        Err(_) => Err(io::Error::other(
            "tui panic recovered (terminal state restored)",
        )),
    };

    if let Err(err) = session.restore() {
        if result.is_ok() {
            result = Err(err);
        }
    }

    result
}

fn draw_ui(
    frame: &mut Frame<'_>,
    state: &Arc<Mutex<DashboardState>>,
    stats: &RuntimeStats,
    ui: &mut UiState,
    control_connected: bool,
) {
    let area = frame.area();
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(1),
            Constraint::Length(1),
            Constraint::Length(1),
            Constraint::Min(6),
            Constraint::Length(3),
        ])
        .split(area);

    let mut signal_count = 0_usize;
    let mut update_count = 0_u64;
    let mut uptime_ms = 0_u64;
    let mut selected_signal_id: Option<u16> = None;
    let mut rows = Vec::new();

    if let Ok(guard) = state.lock() {
        let descriptors = guard.descriptors();
        signal_count = descriptors.len();
        update_count = guard.update_count();
        uptime_ms = guard.uptime_ms();

        if signal_count > 0 {
            ui.selected_index = ui.selected_index.min(signal_count - 1);
        } else {
            ui.selected_index = 0;
            ui.scroll = 0;
        }

        selected_signal_id = descriptors
            .get(ui.selected_index)
            .map(|descriptor| descriptor.signal_id);

        let visible_rows = chunks[3].height.saturating_sub(3).max(1) as usize;
        let max_scroll = signal_count.saturating_sub(visible_rows);
        ui.scroll = ui.scroll.min(max_scroll);
        if signal_count > 0 {
            if ui.selected_index < ui.scroll {
                ui.scroll = ui.selected_index;
            }
            if ui.selected_index >= ui.scroll + visible_rows {
                ui.scroll = ui.selected_index + 1 - visible_rows;
            }
        }

        rows.reserve(visible_rows);
        for (index, descriptor) in descriptors
            .iter()
            .enumerate()
            .skip(ui.scroll)
            .take(visible_rows)
        {
            let value = guard
                .value_for(descriptor.signal_id)
                .map(format_signal_value)
                .unwrap_or_else(|| String::from("-"));

            let mut row = Row::new(vec![
                descriptor.signal_id.to_string(),
                descriptor.signal_type.as_str().to_string(),
                format!(
                    "{}/{}",
                    descriptor.kind.as_str(),
                    descriptor.access.as_str()
                ),
                descriptor.path.clone(),
                value,
            ]);
            if index == ui.selected_index {
                row = row.style(Style::default().add_modifier(Modifier::REVERSED));
            }
            rows.push(row);
        }
    }

    let cpu_text = stats
        .cpu_percent
        .map(|value| format!("{value:.1}%"))
        .unwrap_or_else(|| String::from("n/a"));
    let rss_text = stats
        .rss_bytes
        .map(format_memory)
        .unwrap_or_else(|| String::from("n/a"));

    let top_line = format!(
        "ARCP CLI TUI | control={} signals={} updates={} uptimeMs={} cpu={} rss={}",
        if control_connected { "up" } else { "down" },
        signal_count,
        update_count,
        uptime_ms,
        cpu_text,
        rss_text
    );
    frame.render_widget(Paragraph::new(top_line), chunks[0]);

    let selected_display = selected_signal_id
        .map(|id| id.to_string())
        .unwrap_or_else(|| String::from("-"));
    frame.render_widget(
        Paragraph::new(format!(
            "status: {} | selected={}",
            ui.status, selected_display
        )),
        chunks[1],
    );

    frame.render_widget(
        Paragraph::new(
            "keys: ? help | p ping | a action selected | s set selected | Up/Down select | Esc quit",
        ),
        chunks[2],
    );

    let table = Table::new(
        rows,
        [
            Constraint::Length(6),
            Constraint::Length(9),
            Constraint::Length(17),
            Constraint::Percentage(40),
            Constraint::Percentage(40),
        ],
    )
    .header(
        Row::new(vec!["ID", "TYPE", "KIND/ACCESS", "PATH", "VALUE"])
            .style(Style::default().add_modifier(Modifier::BOLD)),
    )
    .block(
        Block::default()
            .borders(Borders::ALL)
            .title(format!("Signals (selected index {})", ui.selected_index)),
    );
    frame.render_widget(table, chunks[3]);

    let input_display = if ui.input.is_empty() {
        String::from("> (type help or press ?)")
    } else {
        format!("> {}", ui.input)
    };
    let command = Paragraph::new(input_display)
        .block(Block::default().borders(Borders::ALL).title("Command"));
    frame.render_widget(command, chunks[4]);

    if ui.show_help {
        let popup = centered_rect(88, 74, area);
        frame.render_widget(Clear, popup);
        frame.render_widget(
            Paragraph::new(HELP_TEXT).block(Block::default().title("Help").borders(Borders::ALL)),
            popup,
        );
    }
}

fn centered_rect(percent_x: u16, percent_y: u16, area: Rect) -> Rect {
    let vertical = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Percentage((100_u16.saturating_sub(percent_y)) / 2),
            Constraint::Percentage(percent_y),
            Constraint::Percentage((100_u16.saturating_sub(percent_y)) / 2),
        ])
        .split(area);
    Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage((100_u16.saturating_sub(percent_x)) / 2),
            Constraint::Percentage(percent_x),
            Constraint::Percentage((100_u16.saturating_sub(percent_x)) / 2),
        ])
        .split(vertical[1])[1]
}

fn handle_key(
    key: KeyEvent,
    ui: &mut UiState,
    control_link: &mut ControlLink,
    state: &Arc<Mutex<DashboardState>>,
) -> io::Result<bool> {
    let descriptor_count = descriptor_count(state);
    let selected_id = selected_signal_id(state, ui.selected_index);

    match key.code {
        KeyCode::Esc => return Ok(true),
        KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => return Ok(true),
        KeyCode::Char('u') if key.modifiers.contains(KeyModifiers::CONTROL) => {
            ui.input.clear();
            return Ok(false);
        }
        KeyCode::Char('?') => {
            ui.show_help = !ui.show_help;
            return Ok(false);
        }
        KeyCode::Char('p') if ui.input.is_empty() => {
            ui.status = execute_command(
                "ping",
                selected_id,
                control_link,
                state,
                &mut ui.selected_index,
            );
            return Ok(false);
        }
        KeyCode::Char('a') if ui.input.is_empty() => {
            ui.status = execute_command(
                "action",
                selected_id,
                control_link,
                state,
                &mut ui.selected_index,
            );
            return Ok(false);
        }
        KeyCode::Char('s') if ui.input.is_empty() => {
            if let Some(signal_id) = selected_id {
                ui.input = format!("set {signal_id} ");
                ui.status = String::from("edit value then press Enter");
            } else {
                ui.status = String::from("no signal selected");
            }
            return Ok(false);
        }
        KeyCode::Char(ch) => ui.input.push(ch),
        KeyCode::Backspace => {
            ui.input.pop();
        }
        KeyCode::Enter => {
            let command = ui.input.trim().to_string();
            ui.input.clear();
            if command.eq_ignore_ascii_case("quit")
                || command.eq_ignore_ascii_case("exit")
                || command.eq_ignore_ascii_case("q")
            {
                return Ok(true);
            }
            ui.status = execute_command(
                &command,
                selected_id,
                control_link,
                state,
                &mut ui.selected_index,
            );
        }
        KeyCode::Up => {
            if ui.input.is_empty() && ui.selected_index > 0 {
                ui.selected_index -= 1;
            }
        }
        KeyCode::Down => {
            if ui.input.is_empty() && ui.selected_index + 1 < descriptor_count {
                ui.selected_index += 1;
            }
        }
        KeyCode::PageUp => {
            if ui.input.is_empty() {
                ui.selected_index = ui.selected_index.saturating_sub(10);
            }
        }
        KeyCode::PageDown => {
            if ui.input.is_empty() && descriptor_count > 0 {
                ui.selected_index = (ui.selected_index + 10).min(descriptor_count - 1);
            }
        }
        KeyCode::Home => {
            if ui.input.is_empty() {
                ui.selected_index = 0;
            }
        }
        KeyCode::End => {
            if ui.input.is_empty() && descriptor_count > 0 {
                ui.selected_index = descriptor_count - 1;
            }
        }
        _ => {}
    }

    Ok(false)
}

fn execute_command(
    command: &str,
    selected_id: Option<u16>,
    control_link: &mut ControlLink,
    state: &Arc<Mutex<DashboardState>>,
    selected_index: &mut usize,
) -> String {
    if command.is_empty() {
        return String::from("ready");
    }

    if command.eq_ignore_ascii_case("help") {
        return String::from(
            "help, ping, manifest, action [id], set <id> <value>, set <value> (selected), select <id>, quit",
        );
    }

    if command.eq_ignore_ascii_case("ping") {
        return match control_link.with_client("PING", |client| client.ping()) {
            Ok(()) => String::from("PONG"),
            Err(msg) => msg,
        };
    }

    if command.eq_ignore_ascii_case("manifest") {
        let count = state
            .lock()
            .map(|guard| guard.descriptors().len())
            .unwrap_or(0);
        return format!("manifest contains {count} signals");
    }

    if let Some(rest) = command.strip_prefix("select ") {
        let signal_id = match rest.trim().parse::<u16>() {
            Ok(signal_id) => signal_id,
            Err(_) => return String::from("invalid signal id for select"),
        };
        return match select_signal_by_id(state, selected_index, signal_id) {
            true => format!("selected signal {signal_id}"),
            false => format!("signal {signal_id} not found"),
        };
    }

    if command.eq_ignore_ascii_case("action") {
        let signal_id = match selected_id {
            Some(signal_id) => signal_id,
            None => return String::from("no selected signal for action"),
        };
        return match control_link.with_client("ACTION", |client| client.action(signal_id)) {
            Ok(()) => format!("OK ACTION {signal_id}"),
            Err(msg) => msg,
        };
    }

    if let Some(rest) = command.strip_prefix("action ") {
        let signal_id = match rest.trim().parse::<u16>() {
            Ok(signal_id) => signal_id,
            Err(_) => return String::from("invalid signal id for action"),
        };
        return match control_link.with_client("ACTION", |client| client.action(signal_id)) {
            Ok(()) => format!("OK ACTION {signal_id}"),
            Err(msg) => msg,
        };
    }

    if command.eq_ignore_ascii_case("set") {
        return String::from("usage: set <id> <value> OR set <value> (selected)");
    }

    if let Some(rest) = command.strip_prefix("set ") {
        let (signal_id, value_raw) = match parse_set_command(rest, selected_id) {
            Ok(parsed) => parsed,
            Err(msg) => return msg,
        };
        return match control_link
            .with_client("SET", |client| client.set_value(signal_id, &value_raw))
        {
            Ok(()) => format!("OK SET {signal_id}={value_raw}"),
            Err(msg) => msg,
        };
    }

    format!("unknown command: {command} (type help)")
}

fn parse_set_command(rest: &str, selected_id: Option<u16>) -> Result<(u16, String), String> {
    let trimmed = rest.trim();
    if trimmed.is_empty() {
        return Err(String::from("set requires a value"));
    }

    let mut parts = trimmed.splitn(2, ' ');
    let first = parts.next().unwrap_or_default();
    let second = parts.next();

    if let Some(value) = second {
        if let Ok(signal_id) = first.parse::<u16>() {
            let value = value.trim();
            if value.is_empty() {
                return Err(String::from("set requires a non-empty value"));
            }
            return Ok((signal_id, value.to_string()));
        }
    }

    let signal_id = selected_id.ok_or_else(|| {
        String::from("set <value> needs a selected signal; use arrows or 'select <id>'")
    })?;
    Ok((signal_id, trimmed.to_string()))
}

fn descriptor_count(state: &Arc<Mutex<DashboardState>>) -> usize {
    state
        .lock()
        .map(|guard| guard.descriptors().len())
        .unwrap_or(0)
}

fn selected_signal_id(state: &Arc<Mutex<DashboardState>>, selected_index: usize) -> Option<u16> {
    let guard = state.lock().ok()?;
    guard
        .descriptors()
        .get(selected_index)
        .map(|descriptor| descriptor.signal_id)
}

fn select_signal_by_id(
    state: &Arc<Mutex<DashboardState>>,
    selected_index: &mut usize,
    signal_id: u16,
) -> bool {
    let guard = match state.lock() {
        Ok(guard) => guard,
        Err(_) => return false,
    };
    let position = guard
        .descriptors()
        .iter()
        .position(|descriptor| descriptor.signal_id == signal_id);
    if let Some(position) = position {
        *selected_index = position;
        return true;
    }
    false
}

struct ControlLink {
    client: Option<ControlClient>,
    host: String,
    control_port: u16,
    udp_port: u16,
    next_reconnect_at: Instant,
    next_healthcheck_at: Instant,
}

impl ControlLink {
    fn new(client: ControlClient, host: String, control_port: u16, udp_port: u16) -> Self {
        Self {
            client: Some(client),
            host,
            control_port,
            udp_port,
            next_reconnect_at: Instant::now() + CONTROL_RECONNECT_INTERVAL,
            next_healthcheck_at: Instant::now() + CONTROL_HEALTHCHECK_INTERVAL,
        }
    }

    fn connected(&self) -> bool {
        self.client.is_some()
    }

    fn unsubscribe_best_effort(&mut self) {
        if let Some(client) = self.client.as_mut() {
            let _ = client.unsubscribe(self.udp_port);
        }
    }

    fn with_client<T>(
        &mut self,
        context: &str,
        f: impl FnOnce(&mut ControlClient) -> io::Result<T>,
    ) -> Result<T, String> {
        let client = match self.client.as_mut() {
            Some(client) => client,
            None => {
                return Err(String::from(
                    "control disconnected; waiting for automatic reconnect",
                ));
            }
        };
        match f(client) {
            Ok(value) => Ok(value),
            Err(err) => {
                self.client = None;
                self.next_reconnect_at = Instant::now();
                Err(format!("{context} failed: {err}; reconnecting..."))
            }
        }
    }

    fn maybe_reconnect(&mut self, state: &Arc<Mutex<DashboardState>>) -> Option<String> {
        let now = Instant::now();
        if let Some(client) = self.client.as_mut() {
            if now < self.next_healthcheck_at {
                return None;
            }
            self.next_healthcheck_at = now + CONTROL_HEALTHCHECK_INTERVAL;
            if let Err(err) = client.ping() {
                self.client = None;
                self.next_reconnect_at = now;
                return Some(format!("control link lost: {err}; reconnecting..."));
            }
            return None;
        }
        if now < self.next_reconnect_at {
            return None;
        }
        self.next_reconnect_at = now + CONTROL_RECONNECT_INTERVAL;

        let mut client = match ControlClient::connect(&self.host, self.control_port) {
            Ok(client) => client,
            Err(err) => return Some(format!("reconnect failed: {err}")),
        };
        if let Err(err) = client.ping() {
            return Some(format!("reconnect ping failed: {err}"));
        }
        let manifest = match client.manifest() {
            Ok(manifest) => manifest,
            Err(err) => return Some(format!("reconnect manifest failed: {err}")),
        };
        if let Err(err) = client.subscribe(self.udp_port) {
            return Some(format!("reconnect subscribe failed: {err}"));
        }

        if let Ok(mut guard) = state.lock() {
            guard.replace_descriptors(manifest);
        }
        self.client = Some(client);
        self.next_healthcheck_at = Instant::now() + CONTROL_HEALTHCHECK_INTERVAL;
        Some(format!(
            "reconnected to {}:{} (resubscribed UDP {})",
            self.host, self.control_port, self.udp_port
        ))
    }
}

fn format_memory(bytes: u64) -> String {
    let mib = (bytes as f64) / (1024.0 * 1024.0);
    format!("{mib:.1} MiB")
}

#[derive(Clone, Copy, Default)]
struct RuntimeStats {
    cpu_percent: Option<f32>,
    rss_bytes: Option<u64>,
}

struct StatsSampler {
    last_refresh: Instant,
    prev_proc_jiffies: Option<u64>,
    prev_total_jiffies: Option<u64>,
    stats: RuntimeStats,
    cpu_scale: f64,
}

impl StatsSampler {
    fn new() -> Self {
        Self {
            last_refresh: Instant::now() - STATS_REFRESH_INTERVAL,
            prev_proc_jiffies: None,
            prev_total_jiffies: None,
            stats: RuntimeStats::default(),
            cpu_scale: std::thread::available_parallelism()
                .map(|n| n.get() as f64)
                .unwrap_or(1.0),
        }
    }

    fn maybe_refresh(&mut self) {
        if self.last_refresh.elapsed() < STATS_REFRESH_INTERVAL {
            return;
        }
        self.last_refresh = Instant::now();

        self.stats.rss_bytes = read_rss_bytes();

        let proc_jiffies = read_process_jiffies();
        let total_jiffies = read_total_jiffies();
        if let (Some(proc_now), Some(total_now)) = (proc_jiffies, total_jiffies) {
            if let (Some(proc_prev), Some(total_prev)) =
                (self.prev_proc_jiffies, self.prev_total_jiffies)
            {
                let delta_proc = proc_now.saturating_sub(proc_prev);
                let delta_total = total_now.saturating_sub(total_prev);
                if delta_total > 0 {
                    let cpu = (delta_proc as f64 / delta_total as f64) * self.cpu_scale * 100.0;
                    self.stats.cpu_percent = Some(cpu as f32);
                }
            }
            self.prev_proc_jiffies = Some(proc_now);
            self.prev_total_jiffies = Some(total_now);
        }
    }

    fn snapshot(&self) -> RuntimeStats {
        self.stats
    }
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

struct UiState {
    input: String,
    status: String,
    show_help: bool,
    selected_index: usize,
    scroll: usize,
}

struct TerminalSession {
    terminal: Terminal<CrosstermBackend<Stdout>>,
    active: bool,
}

impl TerminalSession {
    fn enter() -> io::Result<Self> {
        enable_raw_mode()?;

        let mut stdout = io::stdout();
        if let Err(err) = execute!(stdout, EnterAlternateScreen, Hide) {
            let _ = disable_raw_mode();
            return Err(err);
        }

        let backend = CrosstermBackend::new(stdout);
        let mut terminal = match Terminal::new(backend) {
            Ok(terminal) => terminal,
            Err(err) => {
                let mut fallback = io::stdout();
                let _ = execute!(fallback, Show, LeaveAlternateScreen);
                let _ = disable_raw_mode();
                return Err(err);
            }
        };

        if let Err(err) = terminal.clear() {
            let _ = disable_raw_mode();
            let _ = execute!(terminal.backend_mut(), Show, LeaveAlternateScreen);
            return Err(err);
        }

        Ok(Self {
            terminal,
            active: true,
        })
    }

    fn terminal_mut(&mut self) -> &mut Terminal<CrosstermBackend<Stdout>> {
        &mut self.terminal
    }

    fn restore(&mut self) -> io::Result<()> {
        if !self.active {
            return Ok(());
        }

        let mut first_error: Option<io::Error> = None;

        if let Err(err) = disable_raw_mode() {
            first_error = Some(err);
        }
        if let Err(err) = execute!(self.terminal.backend_mut(), Show, LeaveAlternateScreen) {
            if first_error.is_none() {
                first_error = Some(err);
            }
        }
        if let Err(err) = self.terminal.show_cursor() {
            if first_error.is_none() {
                first_error = Some(err);
            }
        }

        self.active = false;
        if let Some(err) = first_error {
            return Err(err);
        }
        Ok(())
    }
}

impl Drop for TerminalSession {
    fn drop(&mut self) {
        let _ = self.restore();
    }
}

const HELP_TEXT: &str = "Quick Start\n\n- Move selection: Up / Down / PageUp / PageDown / Home / End\n- Send action for selected row: press 'a'\n- Set selected row value: press 's', type value, Enter\n- Ping control server: press 'p'\n- Toggle help: press '?'\n- Quit: Esc or Ctrl-C\n\nCommands\n\n- help\n- ping\n- manifest\n- select <id>\n- action [id]\n- set <id> <value>\n- set <value>        (uses selected row)\n- quit\n\nNotes\n\n- For set/action, the signal must be registered with compatible access (`write` / `invoke`).\n- Status line shows last command result or error.\n";
