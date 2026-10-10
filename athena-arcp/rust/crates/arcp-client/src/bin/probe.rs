//! `arcp-probe`: connects to an ARCP robot or simulator and prints what it sees.
//!
//! ```text
//! arcp-probe [HOST] [PORT] [--write PATH=VALUE] [--seconds N]
//! ```
//!
//! PORT defaults to 5808 (the robot control port); the UDP realtime receive port defaults to 5807.
//!
//! Prints the connection, the dictionary size, the server status signals, live values sampled over
//! the given window (default 3 s), and one write with the robot's reply. Without `--write` the probe
//! writes the first writable signal back with its current value, which changes nothing.

use std::process::ExitCode;
use std::sync::mpsc::RecvTimeoutError;
use std::time::{Duration, Instant};

use arcp_client::DEFAULT_CONTROL_PORT;
use arcp_client::{Client, ClientConfig, DictEntry, Event, Rate, Value};
use arcp_core::v2::value::{TYPE_BOOL, TYPE_F64, TYPE_I64, TYPE_STRING};

const LOCK_ROLE: u8 = 7;
const STATUS_PREFIX: &str = "athena/arcp/status/";
const DICTIONARY_WAIT: Duration = Duration::from_secs(3);

struct Args {
    host: String,
    port: u16,
    write: Option<(String, String)>,
    seconds: f64,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        host: "127.0.0.1".into(),
        port: DEFAULT_CONTROL_PORT,
        write: None,
        seconds: 3.0,
    };
    let mut positional = Vec::new();
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--write" => {
                let spec = it.next().ok_or("--write needs PATH=VALUE")?;
                let (path, value) = spec.split_once('=').ok_or("--write needs PATH=VALUE")?;
                args.write = Some((path.to_owned(), value.to_owned()));
            }
            "--seconds" => {
                args.seconds = it
                    .next()
                    .ok_or("--seconds needs a number")?
                    .parse()
                    .map_err(|_| "bad --seconds")?;
            }
            "-h" | "--help" => {
                return Err("usage: arcp-probe [HOST] [PORT] [--write PATH=VALUE] [--seconds N]".into());
            }
            _ => positional.push(arg),
        }
    }
    if let Some(host) = positional.first() {
        args.host = host.clone();
    }
    if let Some(port) = positional.get(1) {
        args.port = port.parse().map_err(|_| format!("bad port {port}"))?;
    }
    Ok(args)
}

fn fmt_value(value: &Value) -> String {
    match value {
        Value::Bool(v) => v.to_string(),
        Value::I64(v) => v.to_string(),
        Value::F64(v) => format!("{v:.4}"),
        Value::Str(v) => format!("{v:?}"),
        Value::BoolArray(v) => format!("{v:?}"),
        Value::I64Array(v) => format!("{v:?}"),
        Value::F64Array(v) => format!("{:?}", v.iter().map(|x| format!("{x:.3}")).collect::<Vec<_>>()),
        Value::StrArray(v) => format!("{v:?}"),
        Value::Struct { schema_id, bytes } => format!("struct(schema={schema_id}, {} bytes)", bytes.len()),
    }
}

/// Parses a command-line value using the signal's dictionary type.
fn parse_value(entry: &DictEntry, text: &str) -> Result<Value, String> {
    let bad = |what: &str| format!("{text:?} is not a valid {what} for {}", entry.path);
    match entry.type_tag {
        TYPE_BOOL => match text {
            "true" | "1" => Ok(Value::Bool(true)),
            "false" | "0" => Ok(Value::Bool(false)),
            _ => Err(bad("bool")),
        },
        TYPE_I64 => text.parse().map(Value::I64).map_err(|_| bad("integer")),
        TYPE_F64 => text.parse().map(Value::F64).map_err(|_| bad("number")),
        TYPE_STRING => Ok(Value::Str(text.to_owned())),
        other => Err(format!(
            "{}: type tag {other} cannot be written from the command line",
            entry.path
        )),
    }
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let target = format!("{}:{}", args.host, args.port);
    let (client, rx) = Client::connect(ClientConfig::new(args.host.clone(), args.port))
        .map_err(|e| format!("connect {target}: {e}"))?;
    let link = client.status();
    println!(
        "connected {} session={} udp={}{}",
        client.control_addr(),
        link.session_id,
        client.udp_port(),
        if link.udp_fallback { " (fallback: configured UDP port in use)" } else { "" }
    );

    // The dictionary arrives right after Hello. Wait for it before subscribing.
    let deadline = Instant::now() + DICTIONARY_WAIT;
    while client.dictionary().entries.is_empty() {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return Err("no dictionary within 3 s".into());
        }
        match rx.recv_timeout(left) {
            Ok(Event::Disconnected { reason }) => return Err(format!("disconnected: {reason}")),
            Ok(_) => {}
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return Err("event channel closed".into()),
        }
    }
    let dictionary = client.dictionary();
    let status_count = dictionary
        .entries
        .iter()
        .filter(|e| e.path.starts_with(STATUS_PREFIX))
        .count();
    println!(
        "dictionary: {} signals ({} writable, {} status), {} struct schemas",
        dictionary.entries.len(),
        dictionary.entries.iter().filter(|e| e.writable()).count(),
        status_count,
        client.schemas().len()
    );

    let sub = client
        .subscribe("**", Rate::Hz(20.0))
        .map_err(|e| format!("subscribe: {e}"))?;
    println!("subscribed \"**\" at 20 Hz (id {sub})");

    // Sample the live stream for the requested window.
    let window = Duration::from_secs_f64(args.seconds.max(0.1));
    let end = Instant::now() + window;
    let (mut frames, mut gaps) = (0u64, 0u64);
    loop {
        let left = end.saturating_duration_since(Instant::now());
        if left.is_zero() {
            break;
        }
        match rx.recv_timeout(left) {
            Ok(Event::Frame(_)) => frames += 1,
            Ok(Event::Gap { .. }) => gaps += 1,
            Ok(Event::Disconnected { reason }) => return Err(format!("disconnected: {reason}")),
            Ok(_) => {}
            Err(RecvTimeoutError::Timeout) => break,
            Err(RecvTimeoutError::Disconnected) => return Err("event channel closed".into()),
        }
    }
    println!("frames in {:.1} s: {frames} (sequence gaps: {gaps})", args.seconds);

    println!("server status:");
    for entry in dictionary.entries.iter().filter(|e| e.path.starts_with(STATUS_PREFIX)) {
        let value = client
            .value(entry.signal_id)
            .map(|v| fmt_value(&v))
            .unwrap_or_else(|| "-".into());
        println!("  {} = {value}", entry.path);
    }
    if let Some(s) = client.server_status() {
        println!(
            "  clients={} bytes/s={:.0} budget={:.0} throttled={} dropped={}",
            s.clients, s.bytes_per_second, s.budget_bytes_per_second, s.throttled, s.dropped_frames
        );
    }

    println!("live values:");
    let mut shown = 0;
    for entry in dictionary.entries.iter().filter(|e| !e.path.starts_with(STATUS_PREFIX)) {
        if shown == 8 {
            break;
        }
        if let Some(value) = client.value(entry.signal_id) {
            let unit = if entry.unit.is_empty() {
                String::new()
            } else {
                format!(" {}", entry.unit)
            };
            println!("  {} = {}{unit}", entry.path, fmt_value(&value));
            shown += 1;
        }
    }

    // Write: an explicit PATH=VALUE, or the first writable non-lock signal echoed back at its value.
    let (entry, value) = match &args.write {
        Some((path, text)) => {
            let entry = dictionary
                .entries
                .iter()
                .find(|e| &e.path == path)
                .ok_or_else(|| format!("no signal {path}"))?;
            (entry.clone(), parse_value(entry, text)?)
        }
        None => {
            let entry = dictionary
                .entries
                .iter()
                .find(|e| e.writable() && e.role != LOCK_ROLE && !e.path.starts_with(STATUS_PREFIX))
                .ok_or("no writable signal in the dictionary")?;
            let current = client
                .value(entry.signal_id)
                .ok_or_else(|| format!("{} has no value yet", entry.path))?;
            (entry.clone(), current)
        }
    };
    println!("write {} <- {}", entry.path, fmt_value(&value));
    let reply = client
        .write(entry.signal_id, value)
        .map_err(|e| format!("write {}: {e}", entry.path))?;
    println!(
        "  reply: accepted={} reason={:?} actual={}",
        reply.accepted,
        reply.reason,
        fmt_value(&reply.actual_value)
    );
    client.close();
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("arcp-probe: {message}");
            ExitCode::FAILURE
        }
    }
}
