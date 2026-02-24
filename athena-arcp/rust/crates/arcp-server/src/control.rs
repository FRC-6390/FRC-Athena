use std::collections::HashMap;
use std::io::{self, BufRead, BufReader, Write};
use std::net::{IpAddr, SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use arcp_core::{RuntimeEvent, SignalAccess, SignalDescriptor, ARCP_PROTOCOL_VERSION};

use crate::parsing::parse_value_for_type;

pub(crate) fn run_control_loop(
    listener: TcpListener,
    subscribers: Arc<Mutex<Vec<SocketAddr>>>,
    descriptors: Arc<Mutex<HashMap<u16, SignalDescriptor>>>,
    layout_store: Arc<Mutex<HashMap<String, String>>>,
    event_tx: mpsc::Sender<RuntimeEvent>,
    running: Arc<AtomicBool>,
) {
    while running.load(Ordering::SeqCst) {
        match listener.accept() {
            Ok((stream, remote)) => {
                let subscribers_for_client = Arc::clone(&subscribers);
                let descriptors_for_client = Arc::clone(&descriptors);
                let layout_store_for_client = Arc::clone(&layout_store);
                let running_for_client = Arc::clone(&running);
                let event_tx_for_client = event_tx.clone();
                let _ = thread::Builder::new()
                    .name("arcp-control-client".to_string())
                    .spawn(move || {
                        let _ = handle_control_client(
                            stream,
                            remote.ip(),
                            subscribers_for_client,
                            descriptors_for_client,
                            layout_store_for_client,
                            event_tx_for_client,
                            running_for_client,
                        );
                    });
            }
            Err(err) if err.kind() == io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(10));
            }
            Err(_) => {
                thread::sleep(Duration::from_millis(20));
            }
        }
    }
}

fn handle_control_client(
    mut stream: TcpStream,
    client_ip: IpAddr,
    subscribers: Arc<Mutex<Vec<SocketAddr>>>,
    descriptors: Arc<Mutex<HashMap<u16, SignalDescriptor>>>,
    layout_store: Arc<Mutex<HashMap<String, String>>>,
    event_tx: mpsc::Sender<RuntimeEvent>,
    running: Arc<AtomicBool>,
) -> io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_millis(200)))?;
    stream.write_all(format!("WELCOME {}\n", ARCP_PROTOCOL_VERSION).as_bytes())?;
    stream.flush()?;

    let stream_reader = stream.try_clone()?;
    let mut reader = BufReader::new(stream_reader);
    let mut stats_sampler = ServerStatsSampler::new();

    while running.load(Ordering::SeqCst) {
        let mut line = String::new();
        match reader.read_line(&mut line) {
            Ok(0) => return Ok(()),
            Ok(_) => {}
            Err(err)
                if err.kind() == io::ErrorKind::TimedOut
                    || err.kind() == io::ErrorKind::WouldBlock =>
            {
                continue;
            }
            Err(err) => return Err(err),
        }

        let command = line.trim();
        if command.is_empty() {
            continue;
        }
        if command.eq_ignore_ascii_case("PING") {
            stream.write_all(b"PONG\n")?;
            stream.flush()?;
            continue;
        }
        if command.eq_ignore_ascii_case("MANIFEST") {
            send_manifest(&mut stream, &descriptors)?;
            continue;
        }
        if command.eq_ignore_ascii_case("STATS") {
            handle_stats(&mut stats_sampler, &mut stream)?;
            continue;
        }

        if let Some(rest) = command.strip_prefix("SUB ") {
            handle_subscribe(rest, client_ip, &subscribers, &mut stream)?;
            continue;
        }
        if let Some(rest) = command.strip_prefix("UNSUB ") {
            handle_unsubscribe(rest, client_ip, &subscribers, &mut stream)?;
            continue;
        }
        if let Some(rest) = command.strip_prefix("SET ") {
            handle_set(rest, &descriptors, &event_tx, &mut stream)?;
            continue;
        }
        if let Some(rest) = command.strip_prefix("ACTION ") {
            handle_action(rest, &descriptors, &event_tx, &mut stream)?;
            continue;
        }
        if let Some(rest) = command.strip_prefix("LAYOUT_SAVE ") {
            handle_layout_save(rest, &layout_store, &mut stream)?;
            continue;
        }
        if let Some(rest) = command.strip_prefix("LAYOUT_LOAD ") {
            handle_layout_load(rest, &layout_store, &mut stream)?;
            continue;
        }
        if command.eq_ignore_ascii_case("LAYOUT_LIST") {
            handle_layout_list(&layout_store, &mut stream)?;
            continue;
        }

        stream.write_all(b"ERR UNKNOWN\n")?;
        stream.flush()?;
    }

    Ok(())
}

fn handle_subscribe(
    port_str: &str,
    client_ip: IpAddr,
    subscribers: &Arc<Mutex<Vec<SocketAddr>>>,
    stream: &mut TcpStream,
) -> io::Result<()> {
    match port_str.trim().parse::<u16>() {
        Ok(port) if port > 0 => {
            let endpoint = SocketAddr::new(client_ip, port);
            if let Ok(mut list) = subscribers.lock() {
                if !list.contains(&endpoint) {
                    list.push(endpoint);
                }
            }
            stream.write_all(b"OK SUB\n")?;
        }
        _ => stream.write_all(b"ERR SUB\n")?,
    }
    stream.flush()
}

fn handle_unsubscribe(
    port_str: &str,
    client_ip: IpAddr,
    subscribers: &Arc<Mutex<Vec<SocketAddr>>>,
    stream: &mut TcpStream,
) -> io::Result<()> {
    match port_str.trim().parse::<u16>() {
        Ok(port) if port > 0 => {
            let endpoint = SocketAddr::new(client_ip, port);
            if let Ok(mut list) = subscribers.lock() {
                list.retain(|addr| addr != &endpoint);
            }
            stream.write_all(b"OK UNSUB\n")?;
        }
        _ => stream.write_all(b"ERR UNSUB\n")?,
    }
    stream.flush()
}

fn handle_set(
    command_data: &str,
    descriptors: &Arc<Mutex<HashMap<u16, SignalDescriptor>>>,
    event_tx: &mpsc::Sender<RuntimeEvent>,
    stream: &mut TcpStream,
) -> io::Result<()> {
    let mut parts = command_data.splitn(2, ' ');
    let id_raw = parts.next().unwrap_or_default();
    let value_raw = parts.next().unwrap_or_default();
    let signal_id = match id_raw.parse::<u16>() {
        Ok(signal_id) => signal_id,
        Err(_) => {
            stream.write_all(b"ERR SET\n")?;
            stream.flush()?;
            return Ok(());
        }
    };

    let descriptor = {
        let map = match descriptors.lock() {
            Ok(map) => map,
            Err(_) => {
                stream.write_all(b"ERR SET\n")?;
                stream.flush()?;
                return Ok(());
            }
        };
        match map.get(&signal_id) {
            Some(descriptor) => descriptor.clone(),
            None => {
                stream.write_all(b"ERR SET\n")?;
                stream.flush()?;
                return Ok(());
            }
        }
    };

    if descriptor.access != SignalAccess::Write {
        stream.write_all(b"ERR SET\n")?;
        stream.flush()?;
        return Ok(());
    }

    let parsed = match parse_value_for_type(descriptor.signal_type, value_raw) {
        Ok(value) => value,
        Err(_) => {
            stream.write_all(b"ERR SET\n")?;
            stream.flush()?;
            return Ok(());
        }
    };
    let _ = event_tx.send(RuntimeEvent::TunableSet {
        signal_id,
        value: parsed,
    });
    stream.write_all(b"OK SET\n")?;
    stream.flush()
}

fn handle_action(
    command_data: &str,
    descriptors: &Arc<Mutex<HashMap<u16, SignalDescriptor>>>,
    event_tx: &mpsc::Sender<RuntimeEvent>,
    stream: &mut TcpStream,
) -> io::Result<()> {
    let signal_id = match command_data.trim().parse::<u16>() {
        Ok(signal_id) => signal_id,
        Err(_) => {
            stream.write_all(b"ERR ACTION\n")?;
            stream.flush()?;
            return Ok(());
        }
    };

    let descriptor = {
        let map = match descriptors.lock() {
            Ok(map) => map,
            Err(_) => {
                stream.write_all(b"ERR ACTION\n")?;
                stream.flush()?;
                return Ok(());
            }
        };
        match map.get(&signal_id) {
            Some(descriptor) => descriptor.clone(),
            None => {
                stream.write_all(b"ERR ACTION\n")?;
                stream.flush()?;
                return Ok(());
            }
        }
    };

    if descriptor.access != SignalAccess::Invoke {
        stream.write_all(b"ERR ACTION\n")?;
        stream.flush()?;
        return Ok(());
    }
    let _ = event_tx.send(RuntimeEvent::Action { signal_id });
    stream.write_all(b"OK ACTION\n")?;
    stream.flush()
}

fn send_manifest(
    stream: &mut TcpStream,
    descriptors: &Arc<Mutex<HashMap<u16, SignalDescriptor>>>,
) -> io::Result<()> {
    let mut list = match descriptors.lock() {
        Ok(map) => map.values().cloned().collect::<Vec<_>>(),
        Err(_) => {
            stream.write_all(b"ERR MANIFEST\n")?;
            stream.flush()?;
            return Ok(());
        }
    };
    list.sort_by_key(|descriptor| descriptor.signal_id);

    stream.write_all(format!("MANIFEST_BEGIN {}\n", list.len()).as_bytes())?;
    for descriptor in list {
        stream.write_all(
            format!(
                "ITEM {} {} {} {} {} {} {} {} {} {}\n",
                descriptor.signal_id,
                descriptor.signal_type.as_str(),
                descriptor.kind.as_str(),
                descriptor.access.as_str(),
                descriptor.policy.as_str(),
                descriptor.durability.as_str(),
                descriptor.metadata_version,
                descriptor.metadata_hash,
                descriptor.persistence_scope.as_str(),
                descriptor.path
            )
            .as_bytes(),
        )?;
    }
    stream.write_all(b"MANIFEST_END\n")?;
    stream.flush()
}

const MAX_LAYOUT_BYTES: usize = 512 * 1024;
const MAX_LAYOUTS: usize = 64;

fn handle_layout_save(
    command_data: &str,
    layout_store: &Arc<Mutex<HashMap<String, String>>>,
    stream: &mut TcpStream,
) -> io::Result<()> {
    let mut parts = command_data.splitn(2, ' ');
    let layout_name = parts.next().unwrap_or_default().trim();
    let encoded = parts.next().unwrap_or_default().trim();
    if !is_valid_layout_name(layout_name) || encoded.is_empty() {
        stream.write_all(b"ERR LAYOUT_SAVE\n")?;
        stream.flush()?;
        return Ok(());
    }

    let payload_bytes = match hex_decode(encoded) {
        Ok(payload) => payload,
        Err(_) => {
            stream.write_all(b"ERR LAYOUT_SAVE\n")?;
            stream.flush()?;
            return Ok(());
        }
    };
    if payload_bytes.is_empty() || payload_bytes.len() > MAX_LAYOUT_BYTES {
        stream.write_all(b"ERR LAYOUT_SAVE\n")?;
        stream.flush()?;
        return Ok(());
    }
    let payload = match String::from_utf8(payload_bytes) {
        Ok(payload) => payload,
        Err(_) => {
            stream.write_all(b"ERR LAYOUT_SAVE\n")?;
            stream.flush()?;
            return Ok(());
        }
    };

    let mut store = match layout_store.lock() {
        Ok(store) => store,
        Err(_) => {
            stream.write_all(b"ERR LAYOUT_SAVE\n")?;
            stream.flush()?;
            return Ok(());
        }
    };
    if !store.contains_key(layout_name) && store.len() >= MAX_LAYOUTS {
        stream.write_all(b"ERR LAYOUT_SAVE\n")?;
        stream.flush()?;
        return Ok(());
    }
    store.insert(layout_name.to_string(), payload);

    stream.write_all(b"OK LAYOUT_SAVE\n")?;
    stream.flush()
}

fn handle_layout_load(
    command_data: &str,
    layout_store: &Arc<Mutex<HashMap<String, String>>>,
    stream: &mut TcpStream,
) -> io::Result<()> {
    let layout_name = command_data.trim();
    if !is_valid_layout_name(layout_name) {
        stream.write_all(b"ERR LAYOUT_LOAD\n")?;
        stream.flush()?;
        return Ok(());
    }

    let payload = {
        let store = match layout_store.lock() {
            Ok(store) => store,
            Err(_) => {
                stream.write_all(b"ERR LAYOUT_LOAD\n")?;
                stream.flush()?;
                return Ok(());
            }
        };
        match store.get(layout_name) {
            Some(payload) => payload.clone(),
            None => {
                stream.write_all(b"ERR LAYOUT_LOAD\n")?;
                stream.flush()?;
                return Ok(());
            }
        }
    };

    let encoded = hex_encode(payload.as_bytes());
    stream.write_all(format!("LAYOUT_DATA {layout_name} {encoded}\n").as_bytes())?;
    stream.flush()
}

fn handle_layout_list(
    layout_store: &Arc<Mutex<HashMap<String, String>>>,
    stream: &mut TcpStream,
) -> io::Result<()> {
    let mut names = match layout_store.lock() {
        Ok(store) => store.keys().cloned().collect::<Vec<_>>(),
        Err(_) => {
            stream.write_all(b"ERR LAYOUT_LIST\n")?;
            stream.flush()?;
            return Ok(());
        }
    };
    names.sort_unstable();

    stream.write_all(format!("LAYOUT_LIST {} {}\n", names.len(), names.join(" ")).as_bytes())?;
    stream.flush()
}

fn is_valid_layout_name(name: &str) -> bool {
    if name.is_empty() || name.len() > 64 {
        return false;
    }
    name.bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.'))
}

fn hex_encode(raw: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(raw.len().saturating_mul(2));
    for byte in raw {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn hex_decode(raw: &str) -> Result<Vec<u8>, &'static str> {
    if raw.len() % 2 != 0 {
        return Err("hex payload length must be even");
    }
    let mut out = Vec::with_capacity(raw.len() / 2);
    let bytes = raw.as_bytes();
    let mut index = 0_usize;
    while index < bytes.len() {
        let hi = decode_hex_nibble(bytes[index])?;
        let lo = decode_hex_nibble(bytes[index + 1])?;
        out.push((hi << 4) | lo);
        index += 2;
    }
    Ok(out)
}

fn decode_hex_nibble(byte: u8) -> Result<u8, &'static str> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        b'A'..=b'F' => Ok(byte - b'A' + 10),
        _ => Err("invalid hex nibble"),
    }
}

#[derive(Default)]
struct ServerStatsSampler {
    prev_proc_jiffies: Option<u64>,
    prev_total_jiffies: Option<u64>,
    cpu_scale: f64,
    last_sample: Option<Instant>,
}

impl ServerStatsSampler {
    fn new() -> Self {
        Self {
            prev_proc_jiffies: None,
            prev_total_jiffies: None,
            cpu_scale: std::thread::available_parallelism()
                .map(|v| v.get() as f64)
                .unwrap_or(1.0),
            last_sample: None,
        }
    }

    fn sample(&mut self) -> (Option<f32>, Option<u64>) {
        let rss = read_rss_bytes();
        let proc_jiffies = read_process_jiffies();
        let total_jiffies = read_total_jiffies();

        let mut cpu_percent = None;
        if let (Some(proc_now), Some(total_now)) = (proc_jiffies, total_jiffies) {
            if let (Some(proc_prev), Some(total_prev), Some(last)) = (
                self.prev_proc_jiffies,
                self.prev_total_jiffies,
                self.last_sample,
            ) {
                let delta_proc = proc_now.saturating_sub(proc_prev);
                let delta_total = total_now.saturating_sub(total_prev);
                if delta_total > 0 && last.elapsed() >= Duration::from_millis(120) {
                    let cpu = (delta_proc as f64 / delta_total as f64) * self.cpu_scale * 100.0;
                    cpu_percent = Some(cpu as f32);
                }
            }
            self.prev_proc_jiffies = Some(proc_now);
            self.prev_total_jiffies = Some(total_now);
            self.last_sample = Some(Instant::now());
        }

        (cpu_percent, rss)
    }
}

fn handle_stats(stats: &mut ServerStatsSampler, stream: &mut TcpStream) -> io::Result<()> {
    let (cpu_percent, rss_bytes) = stats.sample();
    let cpu = cpu_percent
        .map(|value| format!("{value:.2}"))
        .unwrap_or_else(|| String::from("na"));
    let rss = rss_bytes
        .map(|value| value.to_string())
        .unwrap_or_else(|| String::from("na"));
    stream.write_all(format!("STATS cpu={cpu} rss={rss}\n").as_bytes())?;
    stream.flush()
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
