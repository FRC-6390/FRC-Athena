//! Browser bridge: a WebSocket front for [`Client`] (dashboard in a browser, `bun run dev`).
//!
//! One robot session is shared by every WebSocket client. The protocol is:
//!
//! * **Commands** (client to bridge, text): `{"id": n, "cmd": "...", ...}`. The bridge answers with
//!   `{"id": n, "ok": true, "result": ...}` or `{"id": n, "ok": false, "error": "..."}`.
//!   Commands: `connect {host, port?, replay?}`, `disconnect`, `status`, `subscribe {pattern, hz?}`,
//!   `unsubscribe {subscriptionId}`, `write {signalId | path, value}`, `action {signalId | path, run?}`,
//!   `timeSync {samples?}`, `schemaRequest`, `fileList`, `fileDownload {name, maxBytes?}` (base64
//!   result), `fileDelete {name}`.
//! * **Events** (bridge to client, text, no id): `{"event": "status" | "gap" | "backfill" |
//!   "disconnected", ...}`.
//! * **Records** (bridge to client, binary): E1-W record stream, the same bytes the Tauri bridge
//!   emits as `live://records`. Batched about every 16 ms. A new client first receives the current
//!   dictionary and struct schemas.
//!
//! The bridge accepts WebSocket upgrades only from loopback origins (or an origin given with
//! `allow_origin`), so an arbitrary web page cannot drive a robot through the visitor's browser.

use std::io;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use arcp_core::v2::record::{encode_stream, Record};
use arcp_core::v2::value::{
    TYPE_BOOL, TYPE_BOOL_ARRAY, TYPE_F64, TYPE_F64_ARRAY, TYPE_I64, TYPE_I64_ARRAY, TYPE_STRING, TYPE_STRING_ARRAY,
};
use arcp_core::v2::Value;
use base64::Engine;
use serde_json::{json, Map, Value as Json};
use tungstenite::handshake::server::{ErrorResponse, Request, Response};
use tungstenite::http::StatusCode;
use tungstenite::Message;

use crate::client::DEFAULT_CONTROL_PORT;
use crate::{Client, ClientConfig, ClientError, Event, LinkStatus, Rate, ServerStatus};

pub const DEFAULT_LISTEN: &str = "127.0.0.1:5811";
const BATCH_INTERVAL: Duration = Duration::from_millis(16);
const STATUS_INTERVAL: Duration = Duration::from_millis(500);
const READ_POLL: Duration = Duration::from_millis(5);
const DEFAULT_MAX_DOWNLOAD: usize = 16 * 1024 * 1024;

/// Messages queued for one WebSocket client.
#[derive(Clone)]
enum Outgoing {
    Text(String),
    Binary(Vec<u8>),
}

/// Shared bridge state: at most one robot session, and every connected WebSocket client.
struct Bridge {
    session: Mutex<Option<Session>>,
    subscribers: Mutex<Vec<Sender<Outgoing>>>,
    generation: AtomicU64,
    allow_origins: Vec<String>,
}

#[derive(Clone)]
struct Session {
    generation: u64,
    client: Client,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

impl Bridge {
    fn current(&self) -> Option<Session> {
        lock(&self.session).clone()
    }

    fn is_current(&self, generation: u64) -> bool {
        lock(&self.session).as_ref().is_some_and(|s| s.generation == generation)
    }

    fn broadcast(&self, message: Outgoing) {
        lock(&self.subscribers).retain(|tx| tx.send(message.clone()).is_ok());
    }

    fn event(&self, value: Json) {
        self.broadcast(Outgoing::Text(value.to_string()));
    }
}

/// Runs the bridge on `listener` until the process ends. `allow_origins` lists extra exact origins
/// (for example `https://dashboard.example`) that may connect; loopback origins are always allowed.
pub fn serve(listener: TcpListener, allow_origins: Vec<String>) -> io::Result<()> {
    let bridge = Arc::new(Bridge {
        session: Mutex::new(None),
        subscribers: Mutex::new(Vec::new()),
        generation: AtomicU64::new(0),
        allow_origins,
    });
    for stream in listener.incoming() {
        let stream = stream?;
        let bridge = Arc::clone(&bridge);
        thread::Builder::new().name("ws-bridge-client".into()).spawn(move || {
            let _ = stream.set_nodelay(true);
            serve_client(&bridge, stream);
        })?;
    }
    Ok(())
}

/// True for origins a local dashboard runs from: loopback hosts, `*.localhost` (Tauri uses
/// `http://tauri.localhost`), or an exact entry of `allow_origins`. A missing Origin (CLI tools,
/// tests) is allowed.
pub fn origin_allowed(origin: Option<&str>, allow_origins: &[String]) -> bool {
    let Some(origin) = origin else {
        return true;
    };
    if allow_origins.iter().any(|allowed| allowed == origin) {
        return true;
    }
    let Some((_, rest)) = origin.split_once("://") else {
        return false;
    };
    let authority = rest.split('/').next().unwrap_or("");
    let host = if let Some(stripped) = authority.strip_prefix('[') {
        stripped.split(']').next().unwrap_or("")
    } else {
        authority.split(':').next().unwrap_or("")
    };
    host == "localhost" || host == "127.0.0.1" || host == "::1" || host.ends_with(".localhost")
}

fn serve_client(bridge: &Arc<Bridge>, stream: TcpStream) {
    let allow = bridge.allow_origins.clone();
    let callback = move |request: &Request, response: Response| {
        let origin = request.headers().get("origin").and_then(|v| v.to_str().ok());
        if origin_allowed(origin, &allow) {
            Ok(response)
        } else {
            let mut refused = ErrorResponse::new(Some("origin not allowed".to_owned()));
            *refused.status_mut() = StatusCode::FORBIDDEN;
            Err(refused)
        }
    };
    let mut socket = match tungstenite::accept_hdr(stream, callback) {
        Ok(socket) => socket,
        Err(_) => return,
    };
    if socket.get_ref().set_read_timeout(Some(READ_POLL)).is_err() {
        return;
    }

    let (tx, rx) = mpsc::channel::<Outgoing>();
    // Register before the snapshot so a dictionary announced in between is not missed.
    lock(&bridge.subscribers).push(tx.clone());
    if let Some(session) = bridge.current() {
        let dictionary = session.client.dictionary();
        let mut records: Vec<Record> = vec![Record::Dictionary(dictionary)];
        records.extend(session.client.schemas().into_iter().map(Record::StructSchema));
        let _ = tx.send(Outgoing::Binary(encode_stream(&records)));
    }

    loop {
        while let Ok(message) = rx.try_recv() {
            let sent = match message {
                Outgoing::Text(text) => socket.send(Message::Text(text.into())),
                Outgoing::Binary(bytes) => socket.send(Message::Binary(bytes.into())),
            };
            if sent.is_err() {
                return;
            }
        }
        match socket.read() {
            Ok(Message::Text(text)) => {
                let reply = handle_text(bridge, text.as_str());
                if tx.send(Outgoing::Text(reply)).is_err() {
                    return;
                }
            }
            Ok(Message::Close(_)) => return,
            Ok(_) => {}
            Err(tungstenite::Error::Io(e))
                if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {}
            Err(_) => return,
        }
    }
}

fn handle_text(bridge: &Arc<Bridge>, text: &str) -> String {
    let request: Json = match serde_json::from_str(text) {
        Ok(v) => v,
        Err(e) => return json!({ "id": Json::Null, "ok": false, "error": format!("bad JSON: {e}") }).to_string(),
    };
    let id = request.get("id").cloned().unwrap_or(Json::Null);
    match dispatch(bridge, &request) {
        Ok(result) => json!({ "id": id, "ok": true, "result": result }).to_string(),
        Err(error) => json!({ "id": id, "ok": false, "error": error }).to_string(),
    }
}

fn str_field<'a>(request: &'a Json, key: &str) -> Result<&'a str, String> {
    request
        .get(key)
        .and_then(Json::as_str)
        .ok_or_else(|| format!("missing string field {key}"))
}

fn u64_field(request: &Json, key: &str) -> Result<u64, String> {
    request
        .get(key)
        .and_then(Json::as_u64)
        .ok_or_else(|| format!("missing integer field {key}"))
}

fn session(bridge: &Bridge) -> Result<Session, String> {
    match bridge.current() {
        Some(s) if s.client.is_connected() => Ok(s),
        _ => Err("not connected to a robot".into()),
    }
}

/// Signal id from `signalId`, or looked up from `path`.
fn signal_of(client: &Client, request: &Json) -> Result<u64, String> {
    if let Some(id) = request.get("signalId").and_then(Json::as_u64) {
        return Ok(id);
    }
    let path = str_field(request, "path")?;
    client.signal_id(path).ok_or_else(|| format!("unknown signal {path}"))
}

fn dispatch(bridge: &Arc<Bridge>, request: &Json) -> Result<Json, String> {
    let cmd = str_field(request, "cmd")?;
    match cmd {
        "connect" => {
            let host = str_field(request, "host")?.to_owned();
            let port = request
                .get("port")
                .and_then(Json::as_u64)
                .map_or(Ok(DEFAULT_CONTROL_PORT), |p| {
                    u16::try_from(p).map_err(|_| format!("bad port {p}"))
                })?;
            let mut config = ClientConfig::new(host, port);
            config.replay = request.get("replay").and_then(Json::as_bool).unwrap_or(false);
            let (client, rx) = Client::connect(config).map_err(|e| e.to_string())?;
            let generation = bridge.generation.fetch_add(1, Ordering::SeqCst) + 1;
            let info = json!({
                "sessionId": client.status().session_id,
                "udpPort": client.udp_port(),
                "controlAddr": client.control_addr().to_string(),
            });
            let previous = lock(&bridge.session).replace(Session {
                generation,
                client: client.clone(),
            });
            if let Some(previous) = previous {
                previous.client.close();
            }
            spawn_pump(Arc::clone(bridge), generation, rx, client)?;
            Ok(info)
        }
        "disconnect" => {
            let previous = lock(&bridge.session).take();
            if let Some(previous) = previous {
                previous.client.close();
                bridge.event(json!({ "event": "disconnected", "reason": "disconnected by client" }));
            }
            Ok(Json::Null)
        }
        "status" => {
            let (link, server) = match bridge.current() {
                Some(s) => (s.client.status(), s.client.server_status()),
                None => (LinkStatus::default(), None),
            };
            Ok(link_json(&link, server.as_ref()))
        }
        "subscribe" => {
            let s = session(bridge)?;
            let pattern = str_field(request, "pattern")?;
            let rate = match request.get("hz").and_then(Json::as_f64) {
                Some(hz) => Rate::Hz(hz),
                None => Rate::OnChange,
            };
            let id = s.client.subscribe(pattern, rate).map_err(|e| e.to_string())?;
            Ok(json!(id))
        }
        "unsubscribe" => {
            let s = session(bridge)?;
            let id = u64_field(request, "subscriptionId")?;
            s.client.unsubscribe(id as u32).map_err(|e| e.to_string())?;
            Ok(Json::Null)
        }
        "write" => {
            let s = session(bridge)?;
            let signal = signal_of(&s.client, request)?;
            let tag = s
                .client
                .dictionary()
                .entries
                .iter()
                .find(|e| e.signal_id == signal)
                .map(|e| e.type_tag);
            let value = json_to_value(request.get("value").unwrap_or(&Json::Null), tag)?;
            // A rejection is data (accepted false), not an error. The robot decides (E5).
            let reply = s.client.write(signal, value).map_err(|e| e.to_string())?;
            Ok(json!({
                "accepted": reply.accepted,
                "reason": reply.reason,
                "actualValue": value_to_json(&reply.actual_value),
            }))
        }
        "action" => {
            let s = session(bridge)?;
            let signal = signal_of(&s.client, request)?;
            let run = request.get("run").and_then(Json::as_bool).unwrap_or(true);
            let result = if run {
                s.client.action_run(signal)
            } else {
                s.client.action_cancel(signal)
            };
            match result {
                Ok(()) => Ok(json!({ "accepted": true, "reason": "" })),
                Err(ClientError::Server { reason, .. }) => Ok(json!({ "accepted": false, "reason": reason })),
                Err(e) => Err(e.to_string()),
            }
        }
        "timeSync" => {
            let s = session(bridge)?;
            let samples = request.get("samples").and_then(Json::as_u64).unwrap_or(8).clamp(1, 64) as usize;
            let est = s.client.time_sync(samples).map_err(|e| e.to_string())?;
            Ok(json!({ "offsetMicros": est.offset_micros, "rttMicros": est.rtt_micros, "samples": est.samples }))
        }
        "schemaRequest" => {
            session(bridge)?.client.schema_request().map_err(|e| e.to_string())?;
            Ok(Json::Null)
        }
        "fileList" => {
            let files = session(bridge)?.client.file_list().map_err(|e| e.to_string())?;
            Ok(Json::Array(
                files
                    .into_iter()
                    .map(|f| json!({ "name": f.name, "size": f.size, "modifiedUnixSecs": f.modified_unix_secs }))
                    .collect(),
            ))
        }
        "fileDownload" => {
            let s = session(bridge)?;
            let name = str_field(request, "name")?;
            let max = request
                .get("maxBytes")
                .and_then(Json::as_u64)
                .map_or(DEFAULT_MAX_DOWNLOAD, |m| m as usize);
            let bytes = s.client.file_download(name, max).map_err(|e| e.to_string())?;
            Ok(json!(base64::engine::general_purpose::STANDARD.encode(bytes)))
        }
        "fileDelete" => {
            let s = session(bridge)?;
            s.client
                .file_delete(str_field(request, "name")?)
                .map_err(|e| e.to_string())?;
            Ok(Json::Null)
        }
        other => Err(format!("unknown command {other}")),
    }
}

fn spawn_pump(bridge: Arc<Bridge>, generation: u64, rx: Receiver<Event>, client: Client) -> Result<(), String> {
    thread::Builder::new()
        .name("ws-bridge-pump".into())
        .spawn(move || run_pump(&bridge, generation, &rx, &client))
        .map(|_| ())
        .map_err(|e| e.to_string())
}

/// Drains the client's events: records are batched into binary messages, status is pushed as text.
fn run_pump(bridge: &Bridge, generation: u64, rx: &Receiver<Event>, client: &Client) {
    let mut batch: Vec<Record> = Vec::new();
    let mut last_flush = Instant::now();
    let mut last_status = Instant::now() - STATUS_INTERVAL;
    loop {
        let mut ended = None;
        match rx.recv_timeout(BATCH_INTERVAL) {
            Ok(event) => ended = absorb(bridge, event, &mut batch),
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => ended = Some("client closed".to_owned()),
        }
        if last_flush.elapsed() >= BATCH_INTERVAL || ended.is_some() {
            flush(bridge, &mut batch);
            last_flush = Instant::now();
        }
        if last_status.elapsed() >= STATUS_INTERVAL || ended.is_some() {
            let server = client.server_status();
            bridge.event(json!({ "event": "status", "status": link_json(&client.status(), server.as_ref()) }));
            last_status = Instant::now();
        }
        if let Some(reason) = ended {
            // A replaced or closed-by-us session must not report a disconnect for the new one.
            if bridge.is_current(generation) {
                bridge.event(json!({ "event": "disconnected", "reason": reason }));
            }
            return;
        }
    }
}

/// Queues one event. Returns the reason when the connection has ended.
fn absorb(bridge: &Bridge, event: Event, batch: &mut Vec<Record>) -> Option<String> {
    match event {
        Event::Dictionary { dictionary, schemas } => {
            batch.push(Record::Dictionary(dictionary));
            batch.extend(schemas.into_iter().map(Record::StructSchema));
        }
        Event::Frame(frame) => batch.push(Record::Frame(frame)),
        Event::Record(record) => batch.push(record),
        Event::Gap { from, to, lost } => bridge.event(json!({ "event": "gap", "from": from, "to": to, "lost": lost })),
        Event::Backfill { from, to, result } => match result {
            Ok(records) => {
                bridge.event(json!({ "event": "backfill", "from": from, "to": to, "records": records.len() }));
                batch.extend(records);
            }
            Err(error) => bridge.event(json!({ "event": "backfill", "from": from, "to": to, "error": error })),
        },
        Event::Disconnected { reason } => return Some(reason),
    }
    None
}

fn flush(bridge: &Bridge, batch: &mut Vec<Record>) {
    if batch.is_empty() {
        return;
    }
    bridge.broadcast(Outgoing::Binary(encode_stream(batch)));
    batch.clear();
}

fn link_json(link: &LinkStatus, server: Option<&ServerStatus>) -> Json {
    let mut out = Map::new();
    out.insert("connected".into(), json!(link.connected));
    out.insert("sessionId".into(), json!(link.session_id));
    out.insert("rttMicros".into(), json!(link.rtt_micros));
    out.insert("offsetMicros".into(), json!(link.offset_micros));
    out.insert("frames".into(), json!(link.frames));
    out.insert("duplicates".into(), json!(link.duplicates));
    out.insert("gaps".into(), json!(link.gaps));
    out.insert("lostCycles".into(), json!(link.lost_cycles));
    out.insert("droppedIncomplete".into(), json!(link.dropped_incomplete));
    out.insert("backfills".into(), json!(link.backfills));
    out.insert("restarts".into(), json!(link.restarts));
    out.insert("malformedDatagrams".into(), json!(link.malformed_datagrams));
    out.insert("udpFallback".into(), json!(link.udp_fallback));
    out.insert(
        "server".into(),
        server.map_or(Json::Null, |s| {
            json!({
                "clients": s.clients,
                "bytesPerSecond": s.bytes_per_second,
                "budgetBytesPerSecond": s.budget_bytes_per_second,
                "throttled": s.throttled,
                "throttledPatterns": s.throttled_patterns,
                "droppedFrames": s.dropped_frames,
            })
        }),
    );
    Json::Object(out)
}

/// JSON to a typed value. With a dictionary type tag the number is converted to that type, so
/// `3` written to an F64 signal stays a float. Without a tag the JSON type decides.
pub fn json_to_value(json: &Json, tag: Option<u8>) -> Result<Value, String> {
    let bad = || format!("value {json} does not match signal type {}", tag.unwrap_or(0));
    match tag {
        Some(TYPE_BOOL) => json.as_bool().map(Value::Bool).ok_or_else(bad),
        Some(TYPE_I64) => json.as_i64().map(Value::I64).ok_or_else(bad),
        Some(TYPE_F64) => json.as_f64().map(Value::F64).ok_or_else(bad),
        Some(TYPE_STRING) => json.as_str().map(|s| Value::Str(s.to_owned())).ok_or_else(bad),
        Some(TYPE_BOOL_ARRAY) => array(json, bad, |j| j.as_bool()).map(Value::BoolArray),
        Some(TYPE_I64_ARRAY) => array(json, bad, |j| j.as_i64()).map(Value::I64Array),
        Some(TYPE_F64_ARRAY) => array(json, bad, |j| j.as_f64()).map(Value::F64Array),
        Some(TYPE_STRING_ARRAY) => array(json, bad, |j| j.as_str().map(str::to_owned)).map(Value::StrArray),
        Some(other) => Err(format!("signal type {other} cannot be written as JSON")),
        None => match json {
            Json::Bool(b) => Ok(Value::Bool(*b)),
            Json::Number(n) if n.is_i64() => Ok(Value::I64(n.as_i64().unwrap_or(0))),
            Json::Number(n) => n.as_f64().map(Value::F64).ok_or_else(bad),
            Json::String(s) => Ok(Value::Str(s.clone())),
            _ => Err(bad()),
        },
    }
}

fn array<T>(json: &Json, bad: impl Fn() -> String, item: impl Fn(&Json) -> Option<T>) -> Result<Vec<T>, String> {
    json.as_array()
        .ok_or_else(&bad)?
        .iter()
        .map(|j| item(j).ok_or_else(&bad))
        .collect()
}

/// Typed value to JSON. Non-finite floats become null; struct bytes are base64.
pub fn value_to_json(value: &Value) -> Json {
    let float = |f: &f64| if f.is_finite() { json!(f) } else { Json::Null };
    match value {
        Value::Bool(v) => json!(v),
        Value::I64(v) => json!(v),
        Value::F64(v) => float(v),
        Value::Str(v) => json!(v),
        Value::BoolArray(v) => json!(v),
        Value::I64Array(v) => json!(v),
        Value::F64Array(v) => Json::Array(v.iter().map(float).collect()),
        Value::StrArray(v) => json!(v),
        Value::Struct { schema_id, bytes } => json!({
            "schemaId": schema_id,
            "base64": base64::engine::general_purpose::STANDARD.encode(bytes),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origins_are_loopback_or_listed() {
        let none: Vec<String> = Vec::new();
        assert!(origin_allowed(None, &none));
        for ok in [
            "http://localhost:1420",
            "http://127.0.0.1",
            "http://[::1]:5173",
            "tauri://localhost",
            "http://tauri.localhost",
        ] {
            assert!(origin_allowed(Some(ok), &none), "{ok}");
        }
        for bad in [
            "https://evil.example",
            "http://localhost.evil.example",
            "http://192.168.1.5:1420",
            "null",
        ] {
            assert!(!origin_allowed(Some(bad), &none), "{bad}");
        }
        let allow = vec!["https://dash.example".to_owned()];
        assert!(origin_allowed(Some("https://dash.example"), &allow));
        assert!(!origin_allowed(Some("https://dash.example.evil"), &allow));
    }

    #[test]
    fn json_numbers_follow_the_signal_type() {
        assert_eq!(json_to_value(&json!(3), Some(TYPE_F64)).unwrap(), Value::F64(3.0));
        assert_eq!(json_to_value(&json!(3), Some(TYPE_I64)).unwrap(), Value::I64(3));
        assert!(json_to_value(&json!(3.5), Some(TYPE_I64)).is_err());
        assert_eq!(json_to_value(&json!(true), Some(TYPE_BOOL)).unwrap(), Value::Bool(true));
        assert_eq!(
            json_to_value(&json!([1, 2]), Some(TYPE_I64_ARRAY)).unwrap(),
            Value::I64Array(vec![1, 2])
        );
        assert_eq!(json_to_value(&json!(2.5), None).unwrap(), Value::F64(2.5));
        assert!(json_to_value(&json!({"a": 1}), Some(TYPE_F64)).is_err());
    }

    #[test]
    fn non_finite_floats_become_null() {
        assert_eq!(value_to_json(&Value::F64(f64::NAN)), Json::Null);
        assert_eq!(value_to_json(&Value::F64(1.25)), json!(1.25));
    }
}
