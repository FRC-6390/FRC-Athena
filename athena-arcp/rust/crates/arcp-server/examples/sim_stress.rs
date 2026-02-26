use std::thread;
use std::time::{Duration, Instant};

use arcp_core::{SignalAccess, SignalDescriptor, SignalDurability, SignalPolicy, SignalType};
use arcp_server::{ArcpServer, ArcpServerConfig};

#[derive(Clone, Debug)]
struct StressArgs {
    control_port: u16,
    realtime_port: u16,
    signals: u16,
    string_signals: u16,
    f64_array_signals: u16,
    f64_array_len: usize,
    publish_hz: u64,
    updates_per_tick: usize,
    report_ms: u64,
    duration_secs: u64,
}

impl Default for StressArgs {
    fn default() -> Self {
        Self {
            control_port: 5805,
            realtime_port: 5806,
            signals: 600,
            string_signals: 60,
            f64_array_signals: 120,
            f64_array_len: 8,
            publish_hz: 50,
            updates_per_tick: 0,
            report_ms: 1_000,
            duration_secs: 0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum SignalLoadKind {
    F64,
    Str,
    F64Array { len: usize },
}

#[derive(Clone, Debug)]
struct SignalLoadSpec {
    signal_id: u16,
    kind: SignalLoadKind,
}

fn main() {
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(msg) => {
            eprintln!("{msg}");
            std::process::exit(2);
        }
    };

    if let Err(err) = run(args) {
        eprintln!("sim_stress failed: {err}");
        std::process::exit(1);
    }
}

fn run(args: StressArgs) -> Result<(), String> {
    validate_args(&args)?;

    let server = ArcpServer::new(ArcpServerConfig {
        control_port: args.control_port,
        realtime_port: args.realtime_port,
        max_signals: args.signals,
        ..ArcpServerConfig::default()
    });

    let specs = register_signals(&server, &args)?;
    server.start().map_err(|err| err.to_string())?;

    let updates_per_tick = if args.updates_per_tick == 0 {
        specs.len()
    } else {
        args.updates_per_tick.min(specs.len())
    };

    println!(
        "ARCP sim_stress running. control={} realtime={} signals={} (f64={}, string={}, f64[]={}x{})",
        server.control_port(),
        server.realtime_port(),
        args.signals,
        args.signals - args.string_signals - args.f64_array_signals,
        args.string_signals,
        args.f64_array_signals,
        args.f64_array_len
    );
    println!(
        "publish_hz={} updates_per_tick={} (~{} updates/s target)",
        args.publish_hz,
        updates_per_tick,
        updates_per_tick.saturating_mul(args.publish_hz as usize)
    );
    println!(
        "host dashboard: cargo run -p arcp-dashboard --bin arcp-dashboard-cli --release -- --host 127.0.0.1 --control-port {}",
        server.control_port()
    );

    let tick_interval = Duration::from_micros((1_000_000_u64 / args.publish_hz.max(1)).max(1));
    let report_interval = Duration::from_millis(args.report_ms.max(100));
    let started = Instant::now();
    let deadline = if args.duration_secs == 0 {
        None
    } else {
        Some(started + Duration::from_secs(args.duration_secs))
    };

    let mut stats = ProcStatsSampler::new();
    let mut next_signal_index = 0_usize;
    let mut updates_total = 0_u64;
    let mut updates_window = 0_u64;
    let mut bytes_total = 0_u64;
    let mut bytes_window = 0_u64;
    let mut publish_errors_total = 0_u64;
    let mut publish_errors_window = 0_u64;
    let mut last_report = Instant::now();
    let mut tick = 0_u64;

    loop {
        if let Some(stop_at) = deadline {
            if Instant::now() >= stop_at {
                break;
            }
        }

        let loop_started = Instant::now();
        tick = tick.saturating_add(1);

        for _ in 0..updates_per_tick {
            let spec = &specs[next_signal_index];
            next_signal_index = (next_signal_index + 1) % specs.len();

            match publish_signal(&server, spec, tick) {
                Ok(encoded_bytes) => {
                    updates_total = updates_total.saturating_add(1);
                    updates_window = updates_window.saturating_add(1);
                    bytes_total = bytes_total.saturating_add(encoded_bytes as u64);
                    bytes_window = bytes_window.saturating_add(encoded_bytes as u64);
                }
                Err(_) => {
                    publish_errors_total = publish_errors_total.saturating_add(1);
                    publish_errors_window = publish_errors_window.saturating_add(1);
                }
            }
        }

        if last_report.elapsed() >= report_interval {
            let elapsed = last_report.elapsed().as_secs_f64().max(0.001);
            let updates_per_sec = (updates_window as f64 / elapsed) as u64;
            let mib_per_sec = (bytes_window as f64 / elapsed) / (1024.0 * 1024.0);
            let proc_stats = stats.sample();
            let cpu_text = proc_stats
                .cpu_percent
                .map(|value| format!("{value:.1}%"))
                .unwrap_or_else(|| String::from("n/a"));
            let rss_text = proc_stats
                .rss_bytes
                .map(format_memory)
                .unwrap_or_else(|| String::from("n/a"));

            println!(
                "uptime={:>4}s updates/s={:>8} wire={:>7.2} MiB/s cpu={:>6} rss={} errors/s={} total_updates={} total_errors={}",
                started.elapsed().as_secs(),
                updates_per_sec,
                mib_per_sec,
                cpu_text,
                rss_text,
                (publish_errors_window as f64 / elapsed) as u64,
                updates_total,
                publish_errors_total
            );

            updates_window = 0;
            bytes_window = 0;
            publish_errors_window = 0;
            last_report = Instant::now();
        }

        let elapsed = loop_started.elapsed();
        if elapsed < tick_interval {
            thread::sleep(tick_interval - elapsed);
        }
    }

    server.stop();
    println!(
        "sim_stress stopped. runtime={}s total_updates={} total_errors={} total_wire={}",
        started.elapsed().as_secs(),
        updates_total,
        publish_errors_total,
        format_memory(bytes_total)
    );
    Ok(())
}

fn register_signals(server: &ArcpServer, args: &StressArgs) -> Result<Vec<SignalLoadSpec>, String> {
    let mut specs = Vec::with_capacity(args.signals as usize);
    let scalar_signals = args
        .signals
        .saturating_sub(args.string_signals.saturating_add(args.f64_array_signals));

    let mut next_id = 1_u16;

    for index in 0..scalar_signals {
        let signal_id = next_id;
        next_id = next_id.saturating_add(1);

        server
            .register_signal(SignalDescriptor::telemetry(
                signal_id,
                SignalType::F64,
                SignalAccess::Observe,
                SignalPolicy::HighRate,
                SignalDurability::Volatile,
                format!("/Athena/Stress/f64/{index}"),
            ))
            .map_err(|err| err.to_string())?;

        specs.push(SignalLoadSpec {
            signal_id,
            kind: SignalLoadKind::F64,
        });
    }

    for index in 0..args.string_signals {
        let signal_id = next_id;
        next_id = next_id.saturating_add(1);

        server
            .register_signal(SignalDescriptor::telemetry(
                signal_id,
                SignalType::Str,
                SignalAccess::Observe,
                SignalPolicy::OnChange,
                SignalDurability::Retained,
                format!("/Athena/Stress/text/{index}"),
            ))
            .map_err(|err| err.to_string())?;

        specs.push(SignalLoadSpec {
            signal_id,
            kind: SignalLoadKind::Str,
        });
    }

    for index in 0..args.f64_array_signals {
        let signal_id = next_id;
        next_id = next_id.saturating_add(1);

        server
            .register_signal(SignalDescriptor::telemetry(
                signal_id,
                SignalType::F64Array,
                SignalAccess::Observe,
                SignalPolicy::HighRate,
                SignalDurability::Volatile,
                format!("/Athena/Stress/f64_array/{index}"),
            ))
            .map_err(|err| err.to_string())?;

        specs.push(SignalLoadSpec {
            signal_id,
            kind: SignalLoadKind::F64Array {
                len: args.f64_array_len,
            },
        });
    }

    Ok(specs)
}

fn publish_signal(
    server: &ArcpServer,
    spec: &SignalLoadSpec,
    tick: u64,
) -> Result<usize, &'static str> {
    match spec.kind {
        SignalLoadKind::F64 => {
            let phase = (tick as f64 * 0.01) + spec.signal_id as f64 * 0.001;
            let value = phase.sin() * 10.0;
            server.publish_f64(spec.signal_id, value)?;
            Ok(4 + 8)
        }
        SignalLoadKind::Str => {
            let value = format!("signal={} tick={tick}", spec.signal_id);
            let payload_len = 2 + value.len();
            server.publish_string(spec.signal_id, value)?;
            Ok(4 + payload_len)
        }
        SignalLoadKind::F64Array { len } => {
            let mut values = Vec::with_capacity(len);
            for i in 0..len {
                let phase = (tick as f64 * 0.005) + spec.signal_id as f64 * 0.002 + i as f64 * 0.05;
                values.push(phase.sin() * 15.0);
            }
            let payload_len = 2 + len.saturating_mul(8);
            server.publish_f64_array(spec.signal_id, values)?;
            Ok(4 + payload_len)
        }
    }
}

fn parse_args<I>(args: I) -> Result<StressArgs, String>
where
    I: IntoIterator<Item = String>,
{
    let mut out = StressArgs::default();
    let raw: Vec<String> = args.into_iter().collect();
    let mut index = 0_usize;

    while index < raw.len() {
        match raw[index].as_str() {
            "--control-port" => {
                out.control_port =
                    parse_u16(&read_next(&raw, index, "--control-port")?, "--control-port")?;
                index += 2;
            }
            "--realtime-port" => {
                out.realtime_port = parse_u16(
                    &read_next(&raw, index, "--realtime-port")?,
                    "--realtime-port",
                )?;
                index += 2;
            }
            "--signals" => {
                out.signals = parse_u16(&read_next(&raw, index, "--signals")?, "--signals")?;
                index += 2;
            }
            "--string-signals" => {
                out.string_signals = parse_u16(
                    &read_next(&raw, index, "--string-signals")?,
                    "--string-signals",
                )?;
                index += 2;
            }
            "--f64-array-signals" => {
                out.f64_array_signals = parse_u16(
                    &read_next(&raw, index, "--f64-array-signals")?,
                    "--f64-array-signals",
                )?;
                index += 2;
            }
            "--f64-array-len" => {
                out.f64_array_len = parse_usize(
                    &read_next(&raw, index, "--f64-array-len")?,
                    "--f64-array-len",
                )?;
                index += 2;
            }
            "--publish-hz" => {
                out.publish_hz =
                    parse_u64(&read_next(&raw, index, "--publish-hz")?, "--publish-hz")?;
                index += 2;
            }
            "--updates-per-tick" => {
                out.updates_per_tick = parse_usize(
                    &read_next(&raw, index, "--updates-per-tick")?,
                    "--updates-per-tick",
                )?;
                index += 2;
            }
            "--report-ms" => {
                out.report_ms = parse_u64(&read_next(&raw, index, "--report-ms")?, "--report-ms")?;
                index += 2;
            }
            "--duration-secs" => {
                out.duration_secs = parse_u64(
                    &read_next(&raw, index, "--duration-secs")?,
                    "--duration-secs",
                )?;
                index += 2;
            }
            "--help" | "-h" => return Err(usage()),
            unknown => return Err(format!("unknown argument: {unknown}\n{}", usage())),
        }
    }

    validate_args(&out)?;
    Ok(out)
}

fn validate_args(args: &StressArgs) -> Result<(), String> {
    if args.signals == 0 {
        return Err(String::from("--signals must be > 0"));
    }
    if args.publish_hz == 0 {
        return Err(String::from("--publish-hz must be > 0"));
    }
    if args.f64_array_len == 0 {
        return Err(String::from("--f64-array-len must be > 0"));
    }
    if args.string_signals.saturating_add(args.f64_array_signals) > args.signals {
        return Err(String::from(
            "--string-signals + --f64-array-signals must be <= --signals",
        ));
    }
    Ok(())
}

fn read_next(args: &[String], index: usize, name: &str) -> Result<String, String> {
    args.get(index + 1)
        .cloned()
        .ok_or_else(|| format!("missing value for {name}"))
}

fn parse_u16(raw: &str, name: &str) -> Result<u16, String> {
    raw.parse::<u16>()
        .map_err(|_| format!("invalid u16 value for {name}: {raw}"))
}

fn parse_u64(raw: &str, name: &str) -> Result<u64, String> {
    raw.parse::<u64>()
        .map_err(|_| format!("invalid u64 value for {name}: {raw}"))
}

fn parse_usize(raw: &str, name: &str) -> Result<usize, String> {
    raw.parse::<usize>()
        .map_err(|_| format!("invalid usize value for {name}: {raw}"))
}

fn usage() -> String {
    String::from(
        "Usage: sim_stress [--control-port <u16>] [--realtime-port <u16>] [--signals <u16>] \
         [--string-signals <u16>] [--f64-array-signals <u16>] [--f64-array-len <usize>] \
         [--publish-hz <u64>] [--updates-per-tick <usize>] [--report-ms <u64>] [--duration-secs <u64>]",
    )
}

fn format_memory(bytes: u64) -> String {
    let mib = bytes as f64 / (1024.0 * 1024.0);
    format!("{mib:.1} MiB")
}

#[derive(Clone, Copy, Default)]
struct ProcStats {
    cpu_percent: Option<f32>,
    rss_bytes: Option<u64>,
}

struct ProcStatsSampler {
    prev_proc_jiffies: Option<u64>,
    prev_total_jiffies: Option<u64>,
    cpu_scale: f64,
}

impl ProcStatsSampler {
    fn new() -> Self {
        Self {
            prev_proc_jiffies: None,
            prev_total_jiffies: None,
            cpu_scale: std::thread::available_parallelism()
                .map(|n| n.get() as f64)
                .unwrap_or(1.0),
        }
    }

    fn sample(&mut self) -> ProcStats {
        let mut out = ProcStats {
            cpu_percent: None,
            rss_bytes: read_rss_bytes(),
        };

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
                    out.cpu_percent = Some(cpu as f32);
                }
            }
            self.prev_proc_jiffies = Some(proc_now);
            self.prev_total_jiffies = Some(total_now);
        }

        out
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
