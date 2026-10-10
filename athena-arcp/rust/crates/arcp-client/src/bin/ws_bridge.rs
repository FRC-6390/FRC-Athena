//! `arcp-ws-bridge`: WebSocket front for the ARCP client, used by the browser dashboard.
//!
//! ```text
//! arcp-ws-bridge [--listen 127.0.0.1:5811] [--allow-origin ORIGIN]...
//! ```
//!
//! Loopback origins are always accepted. See `arcp_client::ws_bridge` for the protocol.

use std::net::TcpListener;
use std::process::ExitCode;

use arcp_client::ws_bridge::{serve, DEFAULT_LISTEN};

fn main() -> ExitCode {
    let mut listen = DEFAULT_LISTEN.to_owned();
    let mut allow = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--listen" => match args.next() {
                Some(value) => listen = value,
                None => return usage("--listen needs an address"),
            },
            "--allow-origin" => match args.next() {
                Some(value) => allow.push(value),
                None => return usage("--allow-origin needs an origin"),
            },
            _ => return usage(&format!("unknown argument {arg}")),
        }
    }
    let listener = match TcpListener::bind(&listen) {
        Ok(listener) => listener,
        Err(e) => {
            eprintln!("arcp-ws-bridge: cannot listen on {listen}: {e}");
            return ExitCode::FAILURE;
        }
    };
    println!("arcp-ws-bridge listening on ws://{listen}");
    match serve(listener, allow) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("arcp-ws-bridge: {e}");
            ExitCode::FAILURE
        }
    }
}

fn usage(problem: &str) -> ExitCode {
    eprintln!("arcp-ws-bridge: {problem}\nusage: arcp-ws-bridge [--listen ADDR] [--allow-origin ORIGIN]...");
    ExitCode::FAILURE
}
