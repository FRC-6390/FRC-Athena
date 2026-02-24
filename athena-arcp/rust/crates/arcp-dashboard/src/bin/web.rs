use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use arcp_dashboard::{
    parse_web_args, run_http_server, ControlClient, DashboardState, TelemetrySubscription,
    WebServerConfig,
};

fn main() {
    let args = match parse_web_args(std::env::args().skip(1)) {
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

fn run(args: arcp_dashboard::WebArgs) -> io::Result<()> {
    let mut control = ControlClient::connect(&args.host, args.control_port)?;
    control.ping()?;
    let manifest = control.manifest()?;

    let telemetry = TelemetrySubscription::bind_any()?;
    let udp_port = telemetry.local_port()?;
    control.subscribe(udp_port)?;

    let state = Arc::new(Mutex::new(DashboardState::new(manifest)));
    let running = Arc::new(AtomicBool::new(true));
    let telemetry_handle = telemetry.spawn(Arc::clone(&state), Arc::clone(&running))?;

    let shared_control = Arc::new(Mutex::new(control));

    println!(
        "ARCP web dashboard running: http://{}:{}",
        args.http_bind, args.http_port
    );
    println!(
        "Connected to ARCP control={} on {} with UDP subscriber port={}",
        args.control_port, args.host, udp_port
    );

    let result = run_http_server(
        WebServerConfig {
            bind_addr: args.http_bind,
            port: args.http_port,
        },
        Arc::clone(&state),
        Arc::clone(&shared_control),
        Arc::clone(&running),
    );

    running.store(false, Ordering::SeqCst);
    let _ = telemetry_handle.join();

    if let Ok(mut guard) = shared_control.lock() {
        let _ = guard.unsubscribe(udp_port);
    }

    result
}
