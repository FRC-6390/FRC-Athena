use std::collections::HashMap;
use std::io::{self, BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::control::ControlClient;
use crate::json::render_state_json;
use crate::state::DashboardState;

#[derive(Debug, Clone)]
pub struct WebServerConfig {
    pub bind_addr: String,
    pub port: u16,
}

pub fn run_http_server(
    config: WebServerConfig,
    state: Arc<Mutex<DashboardState>>,
    control: Arc<Mutex<ControlClient>>,
    running: Arc<AtomicBool>,
) -> io::Result<()> {
    let listener = TcpListener::bind((config.bind_addr.as_str(), config.port))?;
    listener.set_nonblocking(true)?;

    while running.load(Ordering::SeqCst) {
        match listener.accept() {
            Ok((stream, _peer)) => {
                let _ = handle_connection(stream, Arc::clone(&state), Arc::clone(&control));
            }
            Err(err) if err.kind() == io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(10));
            }
            Err(_) => {
                thread::sleep(Duration::from_millis(20));
            }
        }
    }

    Ok(())
}

fn handle_connection(
    mut stream: TcpStream,
    state: Arc<Mutex<DashboardState>>,
    control: Arc<Mutex<ControlClient>>,
) -> io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_millis(300)))?;

    let mut reader = BufReader::new(stream.try_clone()?);
    let mut request_line = String::new();
    let read = reader.read_line(&mut request_line)?;
    if read == 0 {
        return Ok(());
    }

    loop {
        let mut line = String::new();
        let read = reader.read_line(&mut line)?;
        if read == 0 || line == "\r\n" || line == "\n" {
            break;
        }
    }

    let mut pieces = request_line.split_whitespace();
    let method = pieces.next().unwrap_or_default();
    let target = pieces.next().unwrap_or_default();
    let (path, query) = split_path_and_query(target);

    match (method, path) {
        ("GET", "/") => send_response(
            &mut stream,
            "200 OK",
            "text/html; charset=utf-8",
            INDEX_HTML,
        ),
        ("GET", "/api/state") => {
            let body = {
                let guard = state
                    .lock()
                    .map_err(|_| io::Error::new(io::ErrorKind::Other, "state mutex poisoned"))?;
                render_state_json(&guard)
            };
            send_owned_response(
                &mut stream,
                "200 OK",
                "application/json; charset=utf-8",
                body,
            )
        }
        ("GET", "/health") => {
            send_response(&mut stream, "200 OK", "text/plain; charset=utf-8", "ok\n")
        }
        ("POST", "/api/set") | ("GET", "/api/set") => {
            let params = parse_query_params(query);
            handle_set(&mut stream, &control, &params)
        }
        ("POST", "/api/action") | ("GET", "/api/action") => {
            let params = parse_query_params(query);
            handle_action(&mut stream, &control, &params)
        }
        _ => send_response(
            &mut stream,
            "404 Not Found",
            "application/json; charset=utf-8",
            "{\"ok\":false,\"error\":\"not found\"}",
        ),
    }
}

fn handle_set(
    stream: &mut TcpStream,
    control: &Arc<Mutex<ControlClient>>,
    params: &HashMap<String, String>,
) -> io::Result<()> {
    let signal_id = match params.get("id").and_then(|raw| raw.parse::<u16>().ok()) {
        Some(signal_id) => signal_id,
        None => {
            return send_response(
                stream,
                "400 Bad Request",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"missing or invalid id\"}",
            );
        }
    };

    let value = match params.get("value") {
        Some(value) => value,
        None => {
            return send_response(
                stream,
                "400 Bad Request",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"missing value\"}",
            );
        }
    };

    let result = {
        let mut guard = control
            .lock()
            .map_err(|_| io::Error::new(io::ErrorKind::Other, "control mutex poisoned"))?;
        guard.set_value(signal_id, value)
    };

    match result {
        Ok(()) => send_response(
            stream,
            "200 OK",
            "application/json; charset=utf-8",
            "{\"ok\":true}",
        ),
        Err(err) => send_owned_response(
            stream,
            "500 Internal Server Error",
            "application/json; charset=utf-8",
            format!(
                "{{\"ok\":false,\"error\":\"{}\"}}",
                json_escape(&err.to_string())
            ),
        ),
    }
}

fn handle_action(
    stream: &mut TcpStream,
    control: &Arc<Mutex<ControlClient>>,
    params: &HashMap<String, String>,
) -> io::Result<()> {
    let signal_id = match params.get("id").and_then(|raw| raw.parse::<u16>().ok()) {
        Some(signal_id) => signal_id,
        None => {
            return send_response(
                stream,
                "400 Bad Request",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"missing or invalid id\"}",
            );
        }
    };

    let result = {
        let mut guard = control
            .lock()
            .map_err(|_| io::Error::new(io::ErrorKind::Other, "control mutex poisoned"))?;
        guard.action(signal_id)
    };

    match result {
        Ok(()) => send_response(
            stream,
            "200 OK",
            "application/json; charset=utf-8",
            "{\"ok\":true}",
        ),
        Err(err) => send_owned_response(
            stream,
            "500 Internal Server Error",
            "application/json; charset=utf-8",
            format!(
                "{{\"ok\":false,\"error\":\"{}\"}}",
                json_escape(&err.to_string())
            ),
        ),
    }
}

fn split_path_and_query(target: &str) -> (&str, &str) {
    match target.split_once('?') {
        Some((path, query)) => (path, query),
        None => (target, ""),
    }
}

fn parse_query_params(raw: &str) -> HashMap<String, String> {
    let mut out = HashMap::new();
    for pair in raw.split('&') {
        if pair.is_empty() {
            continue;
        }
        let (key_raw, value_raw) = match pair.split_once('=') {
            Some(parts) => parts,
            None => (pair, ""),
        };
        let key = decode_url_component(key_raw);
        let value = decode_url_component(value_raw);
        out.insert(key, value);
    }
    out
}

fn decode_url_component(raw: &str) -> String {
    let bytes = raw.as_bytes();
    let mut out = String::with_capacity(raw.len());
    let mut index = 0_usize;

    while index < bytes.len() {
        match bytes[index] {
            b'+' => {
                out.push(' ');
                index += 1;
            }
            b'%' if index + 2 < bytes.len() => {
                let hi = decode_hex_nibble(bytes[index + 1]);
                let lo = decode_hex_nibble(bytes[index + 2]);
                if let (Some(hi), Some(lo)) = (hi, lo) {
                    out.push((hi << 4 | lo) as char);
                    index += 3;
                } else {
                    out.push('%');
                    index += 1;
                }
            }
            other => {
                out.push(other as char);
                index += 1;
            }
        }
    }

    out
}

fn decode_hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

fn send_response(
    stream: &mut TcpStream,
    status: &str,
    content_type: &str,
    body: &str,
) -> io::Result<()> {
    send_owned_response(stream, status, content_type, body.to_string())
}

fn send_owned_response(
    stream: &mut TcpStream,
    status: &str,
    content_type: &str,
    body: String,
) -> io::Result<()> {
    let header = format!(
        "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nConnection: close\r\nCache-Control: no-store\r\n\r\n",
        body.as_bytes().len()
    );
    stream.write_all(header.as_bytes())?;
    stream.write_all(body.as_bytes())?;
    stream.flush()
}

fn json_escape(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    for ch in raw.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c => out.push(c),
        }
    }
    out
}

const INDEX_HTML: &str = r#"<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>ARCP Quick Dashboard</title>
  <style>
    :root {
      color-scheme: light;
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      --bg: #f4f4f6;
      --panel: #ffffff;
      --line: #d4d7dd;
      --text: #16181d;
      --muted: #555b66;
      --accent: #0d6bdb;
    }
    body { margin: 0; background: var(--bg); color: var(--text); }
    .wrap { max-width: 1100px; margin: 0 auto; padding: 16px; }
    .panel { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 12px; margin-bottom: 12px; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
    input, button {
      font: inherit;
      padding: 6px 8px;
      border-radius: 7px;
      border: 1px solid var(--line);
      background: #fff;
    }
    button {
      cursor: pointer;
      background: var(--accent);
      border-color: var(--accent);
      color: #fff;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }
    th, td {
      text-align: left;
      border-bottom: 1px solid var(--line);
      padding: 6px;
      vertical-align: top;
    }
    th { color: var(--muted); font-weight: 700; }
    td.value { max-width: 420px; word-break: break-word; }
    #status { color: var(--muted); }
  </style>
</head>
<body>
  <div class=\"wrap\">
    <div class=\"panel\">
      <div class=\"row\">
        <strong>ARCP Quick Dashboard</strong>
        <span id=\"status\">connecting...</span>
      </div>
      <div class=\"row\" style=\"margin-top:8px\">
        <label>Signal ID <input id=\"signalId\" size=\"6\" placeholder=\"5\" /></label>
        <label>Value <input id=\"signalValue\" size=\"18\" placeholder=\"12.5\" /></label>
        <button id=\"setBtn\" type=\"button\">SET</button>
        <button id=\"actionBtn\" type=\"button\">ACTION</button>
      </div>
    </div>
    <div class=\"panel\">
      <table>
        <thead>
          <tr>
            <th>ID</th>
            <th>Path</th>
            <th>Type</th>
            <th>Kind</th>
            <th>Access</th>
            <th>Value</th>
          </tr>
        </thead>
        <tbody id=\"rows\"></tbody>
      </table>
    </div>
  </div>
  <script>
    const rows = document.getElementById('rows');
    const statusEl = document.getElementById('status');
    const idEl = document.getElementById('signalId');
    const valueEl = document.getElementById('signalValue');
    const setBtn = document.getElementById('setBtn');
    const actionBtn = document.getElementById('actionBtn');

    function valueToText(value) {
      if (value === null || value === undefined) return '-';
      if (Array.isArray(value)) return JSON.stringify(value);
      if (typeof value === 'object') return JSON.stringify(value);
      return String(value);
    }

    function renderState(data) {
      rows.textContent = '';
      for (const signal of data.signals) {
        const tr = document.createElement('tr');
        const cols = [signal.id, signal.path, signal.type, signal.kind, signal.access, valueToText(signal.value)];
        cols.forEach((value, index) => {
          const td = document.createElement('td');
          td.textContent = String(value);
          if (index === 5) td.className = 'value';
          tr.appendChild(td);
        });
        rows.appendChild(tr);
      }
      statusEl.textContent = `updates=${data.update_count} uptimeMs=${data.uptime_ms}`;
    }

    async function fetchState() {
      try {
        const response = await fetch('/api/state', { cache: 'no-store' });
        if (!response.ok) throw new Error(`state request failed (${response.status})`);
        const data = await response.json();
        renderState(data);
      } catch (err) {
        statusEl.textContent = `error: ${err.message}`;
      }
    }

    async function invoke(path) {
      try {
        const response = await fetch(path, { method: 'POST' });
        const data = await response.json();
        if (!response.ok || !data.ok) {
          throw new Error(data.error || `request failed (${response.status})`);
        }
        statusEl.textContent = 'command sent';
      } catch (err) {
        statusEl.textContent = `command error: ${err.message}`;
      }
    }

    setBtn.addEventListener('click', () => {
      const id = idEl.value.trim();
      const value = valueEl.value;
      if (!id || value.length === 0) {
        statusEl.textContent = 'set requires id and value';
        return;
      }
      invoke(`/api/set?id=${encodeURIComponent(id)}&value=${encodeURIComponent(value)}`);
    });

    actionBtn.addEventListener('click', () => {
      const id = idEl.value.trim();
      if (!id) {
        statusEl.textContent = 'action requires id';
        return;
      }
      invoke(`/api/action?id=${encodeURIComponent(id)}`);
    });

    fetchState();
    setInterval(fetchState, 250);
  </script>
</body>
</html>
"#;
