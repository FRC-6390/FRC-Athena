//! Live robot connection for the dashboard (F4 data core, Tauri side).
//!
//! Wraps `arcp-client`. Realtime records are re-encoded as E1-W record bytes and sent to the webview
//! in batches of about 16 ms (at most about 60 batches a second), so the existing TypeScript decoder
//! and store are reused unchanged. Status is pushed twice a second. Commands return the robot's
//! replies as they are; the client never blocks a write (E5).

use std::fs::{self, File};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use arcp_client::{Client, ClientConfig, ClientError, Event, LinkStatus, Rate, Record, ServerStatus, Value};
use arcp_core::v2::record::{encode_compressed_block, replay_header};
use arcp_core::v2::Writer;
use base64::Engine;
use serde::{Deserialize, Serialize};
use serde_json::json;
use tauri::{AppHandle, Emitter, Manager, State};

pub const DEFAULT_CONTROL_PORT: u16 = 5810;
/// UI batch period (about 60 Hz).
const BATCH_INTERVAL: Duration = Duration::from_millis(16);
const STATUS_INTERVAL: Duration = Duration::from_millis(500);
/// Recorded records are compressed in blocks of about this many uncompressed bytes.
const RECORDING_BLOCK_BYTES: usize = 64 * 1024;
/// Largest file the webview may download in one request.
const MAX_DOWNLOAD_BYTES: usize = 64 * 1024 * 1024;
/// Robot file that holds the tuning values pulled from the robot.
const TUNING_PULL_FILE: &str = "tuning/pull.txt";
/// Where a pulled tuning file lives inside a robot project.
const PROJECT_TUNING_DIR: &[&str] = &["src", "main", "deploy", "athena", "tuning"];
const PULL_FILE_NAME: &str = "pull.txt";

#[derive(Default)]
pub struct LiveRuntime {
    session: Mutex<Option<LiveSession>>,
}

struct LiveSession {
    client: Client,
    recorder: Arc<Mutex<Option<Recorder>>>,
}

/// Writes records to a `.athrp` file: magic and version, then COMPRESSED_BLOCK records (appendix E1-W).
struct Recorder {
    file: File,
    path: PathBuf,
    pending: Vec<Record>,
    pending_bytes: usize,
}

impl Recorder {
    fn create(path: PathBuf) -> io::Result<Self> {
        let mut file = File::create(&path)?;
        file.write_all(&replay_header())?;
        Ok(Self { file, path, pending: Vec::new(), pending_bytes: 0 })
    }

    fn append(&mut self, records: &[Record]) -> io::Result<()> {
        for record in records {
            let mut w = Writer::new();
            record.encode(&mut w);
            self.pending_bytes += w.len();
            self.pending.push(record.clone());
        }
        if self.pending_bytes >= RECORDING_BLOCK_BYTES {
            self.flush_block()?;
        }
        Ok(())
    }

    fn flush_block(&mut self) -> io::Result<()> {
        if self.pending.is_empty() {
            return Ok(());
        }
        self.file.write_all(&encode_compressed_block(&self.pending))?;
        self.pending.clear();
        self.pending_bytes = 0;
        Ok(())
    }

    fn finish(mut self) -> io::Result<PathBuf> {
        self.flush_block()?;
        self.file.flush()?;
        Ok(self.path)
    }
}

/// Signal value in the webview's JSON form. `kind` names the wire type.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(tag = "kind", content = "value", rename_all = "camelCase")]
pub enum WireValue {
    Bool(bool),
    I64(i64),
    F64(f64),
    Str(String),
    BoolArray(Vec<bool>),
    I64Array(Vec<i64>),
    F64Array(Vec<f64>),
    StrArray(Vec<String>),
    Struct { schema_id: u64, bytes: Vec<u8> },
}

impl From<Value> for WireValue {
    fn from(value: Value) -> Self {
        match value {
            Value::Bool(v) => WireValue::Bool(v),
            Value::I64(v) => WireValue::I64(v),
            Value::F64(v) => WireValue::F64(v),
            Value::Str(v) => WireValue::Str(v),
            Value::BoolArray(v) => WireValue::BoolArray(v),
            Value::I64Array(v) => WireValue::I64Array(v),
            Value::F64Array(v) => WireValue::F64Array(v),
            Value::StrArray(v) => WireValue::StrArray(v),
            Value::Struct { schema_id, bytes } => WireValue::Struct { schema_id, bytes },
        }
    }
}

impl From<WireValue> for Value {
    fn from(value: WireValue) -> Self {
        match value {
            WireValue::Bool(v) => Value::Bool(v),
            WireValue::I64(v) => Value::I64(v),
            WireValue::F64(v) => Value::F64(v),
            WireValue::Str(v) => Value::Str(v),
            WireValue::BoolArray(v) => Value::BoolArray(v),
            WireValue::I64Array(v) => Value::I64Array(v),
            WireValue::F64Array(v) => Value::F64Array(v),
            WireValue::StrArray(v) => Value::StrArray(v),
            WireValue::Struct { schema_id, bytes } => Value::Struct { schema_id, bytes },
        }
    }
}

fn err(e: ClientError) -> String {
    e.to_string()
}

fn client(runtime: &LiveRuntime) -> Result<Client, String> {
    runtime
        .session
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .as_ref()
        .map(|s| s.client.clone())
        .ok_or_else(|| "not connected to a robot".to_owned())
}

fn recorder(runtime: &LiveRuntime) -> Result<Arc<Mutex<Option<Recorder>>>, String> {
    runtime
        .session
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .as_ref()
        .map(|s| Arc::clone(&s.recorder))
        .ok_or_else(|| "not connected to a robot".to_owned())
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectOptions {
    /// `localhost` for the simulator, `10.TE.AM.2` for a robot.
    pub host: String,
    pub control_port: Option<u16>,
    #[serde(default)]
    pub replay: bool,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectInfo {
    pub session_id: u32,
    pub udp_port: u16,
    pub control_addr: String,
}

#[tauri::command]
pub async fn live_connect(
    app: AppHandle,
    runtime: State<'_, LiveRuntime>,
    options: ConnectOptions,
) -> Result<ConnectInfo, String> {
    close_session(&runtime);
    let mut config = ClientConfig::new(options.host, options.control_port.unwrap_or(DEFAULT_CONTROL_PORT));
    config.replay = options.replay;
    let (client, rx) = Client::connect(config).map_err(err)?;
    let info = ConnectInfo {
        session_id: client.status().session_id,
        udp_port: client.udp_port(),
        control_addr: client.control_addr().to_string(),
    };
    let recorder = Arc::new(Mutex::new(None));
    let pump_client = client.clone();
    let pump_recorder = Arc::clone(&recorder);
    let pump_app = app.clone();
    thread::Builder::new()
        .name("live-pump".into())
        .spawn(move || run_pump(pump_app, rx, pump_client, pump_recorder))
        .map_err(|e| e.to_string())?;
    *runtime.session.lock().unwrap_or_else(|p| p.into_inner()) = Some(LiveSession { client, recorder });
    Ok(info)
}

fn close_session(runtime: &LiveRuntime) {
    let old = runtime.session.lock().unwrap_or_else(|p| p.into_inner()).take();
    if let Some(session) = old {
        session.client.close();
        if let Some(rec) = session.recorder.lock().unwrap_or_else(|p| p.into_inner()).take() {
            let _ = rec.finish();
        }
    }
}

#[tauri::command]
pub async fn live_disconnect(runtime: State<'_, LiveRuntime>) -> Result<(), String> {
    close_session(&runtime);
    Ok(())
}

#[tauri::command]
pub async fn live_subscribe(runtime: State<'_, LiveRuntime>, pattern: String, hz: Option<f64>) -> Result<u32, String> {
    let rate = match hz {
        Some(hz) => Rate::Hz(hz),
        None => Rate::OnChange,
    };
    client(&runtime)?.subscribe(&pattern, rate).map_err(err)
}

#[tauri::command]
pub async fn live_unsubscribe(runtime: State<'_, LiveRuntime>, subscription_id: u32) -> Result<(), String> {
    client(&runtime)?.unsubscribe(subscription_id).map_err(err)
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
pub struct WriteReplyView {
    pub accepted: bool,
    pub reason: String,
    pub actual_value: WireValue,
}

/// Sends a write. A rejection is returned as data (`accepted: false`), not as an error.
#[tauri::command]
pub async fn live_write(
    runtime: State<'_, LiveRuntime>,
    signal_id: u64,
    value: WireValue,
) -> Result<WriteReplyView, String> {
    let reply = client(&runtime)?.write(signal_id, value.into()).map_err(err)?;
    Ok(WriteReplyView { accepted: reply.accepted, reason: reply.reason, actual_value: reply.actual_value.into() })
}

#[derive(Serialize)]
pub struct ActionReplyView {
    pub accepted: bool,
    pub reason: String,
}

#[tauri::command]
pub async fn live_action(runtime: State<'_, LiveRuntime>, signal_id: u64, run: bool) -> Result<ActionReplyView, String> {
    let c = client(&runtime)?;
    let result = if run { c.action_run(signal_id) } else { c.action_cancel(signal_id) };
    match result {
        Ok(()) => Ok(ActionReplyView { accepted: true, reason: String::new() }),
        Err(ClientError::Server { reason, .. }) => Ok(ActionReplyView { accepted: false, reason }),
        Err(e) => Err(err(e)),
    }
}

#[tauri::command]
pub async fn live_time_sync(runtime: State<'_, LiveRuntime>, samples: Option<usize>) -> Result<serde_json::Value, String> {
    let est = client(&runtime)?.time_sync(samples.unwrap_or(8)).map_err(err)?;
    Ok(json!({ "offsetMicros": est.offset_micros, "rttMicros": est.rtt_micros, "samples": est.samples }))
}

#[tauri::command]
pub async fn live_schema_request(runtime: State<'_, LiveRuntime>) -> Result<(), String> {
    client(&runtime)?.schema_request().map_err(err)
}

#[tauri::command]
pub async fn live_file_list(runtime: State<'_, LiveRuntime>) -> Result<serde_json::Value, String> {
    let files = client(&runtime)?.file_list().map_err(err)?;
    Ok(serde_json::Value::Array(
        files
            .into_iter()
            .map(|f| json!({ "name": f.name, "size": f.size, "modifiedUnixSecs": f.modified_unix_secs }))
            .collect(),
    ))
}

/// Downloads a robot file and returns its bytes as base64.
#[tauri::command]
pub async fn live_file_download(runtime: State<'_, LiveRuntime>, name: String) -> Result<String, String> {
    let bytes = client(&runtime)?.file_download(&name, MAX_DOWNLOAD_BYTES).map_err(err)?;
    Ok(base64::engine::general_purpose::STANDARD.encode(bytes))
}

#[tauri::command]
pub async fn live_file_delete(runtime: State<'_, LiveRuntime>, name: String) -> Result<(), String> {
    client(&runtime)?.file_delete(&name).map_err(err)
}

fn project_tuning_path(project_dir: &str) -> Result<PathBuf, String> {
    let root = Path::new(project_dir);
    if !root.is_dir() {
        return Err(format!("project folder does not exist: {project_dir}"));
    }
    let mut path = root.to_path_buf();
    for part in PROJECT_TUNING_DIR {
        path.push(part);
    }
    Ok(path.join(PULL_FILE_NAME))
}

/// Saves the robot's `tuning/pull.txt` into the project at `src/main/deploy/athena/tuning/pull.txt`.
#[tauri::command]
pub async fn live_pull_tuning(runtime: State<'_, LiveRuntime>, project_dir: String) -> Result<String, String> {
    let dest = project_tuning_path(&project_dir)?;
    let bytes = client(&runtime)?.file_download(TUNING_PULL_FILE, MAX_DOWNLOAD_BYTES).map_err(err)?;
    if let Some(parent) = dest.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    fs::write(&dest, &bytes).map_err(|e| e.to_string())?;
    Ok(dest.display().to_string())
}

#[derive(Serialize)]
pub struct TuningDiffSource {
    /// Robot's current `tuning/pull.txt`, or `null` when the robot has none.
    pub robot: Option<String>,
    /// The project's last pulled copy, or `null` when none was pulled yet.
    pub project: Option<String>,
}

/// Returns the robot's pull and the project's last pulled copy, so the webview can show a diff.
#[tauri::command]
pub async fn live_tuning_diff(runtime: State<'_, LiveRuntime>, project_dir: String) -> Result<TuningDiffSource, String> {
    let dest = project_tuning_path(&project_dir)?;
    let robot = client(&runtime)?.file_download(TUNING_PULL_FILE, MAX_DOWNLOAD_BYTES).ok();
    let project = fs::read(&dest).ok();
    Ok(TuningDiffSource {
        robot: robot.map(|b| String::from_utf8_lossy(&b).into_owned()),
        project: project.map(|b| String::from_utf8_lossy(&b).into_owned()),
    })
}

#[tauri::command]
pub async fn live_record_start(app: AppHandle, runtime: State<'_, LiveRuntime>) -> Result<String, String> {
    let rec_slot = recorder(&runtime)?;
    let dir = app.path().app_data_dir().map_err(|e| e.to_string())?.join("recordings");
    fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let stamp = SystemTime::now().duration_since(UNIX_EPOCH).map_err(|e| e.to_string())?.as_secs();
    let path = dir.join(format!("live-{stamp}.athrp"));
    let rec = Recorder::create(path.clone()).map_err(|e| e.to_string())?;
    let mut slot = rec_slot.lock().unwrap_or_else(|p| p.into_inner());
    if slot.is_some() {
        return Err("already recording".into());
    }
    *slot = Some(rec);
    Ok(path.display().to_string())
}

#[tauri::command]
pub async fn live_record_stop(runtime: State<'_, LiveRuntime>) -> Result<Option<String>, String> {
    let rec_slot = recorder(&runtime)?;
    let taken = rec_slot.lock().unwrap_or_else(|p| p.into_inner()).take();
    match taken {
        Some(rec) => Ok(Some(rec.finish().map_err(|e| e.to_string())?.display().to_string())),
        None => Ok(None),
    }
}

fn status_json(status: &LinkStatus, server: Option<&ServerStatus>) -> serde_json::Value {
    json!({
        "connected": status.connected,
        "sessionId": status.session_id,
        "rttMicros": status.rtt_micros,
        "offsetMicros": status.offset_micros,
        "frames": status.frames,
        "duplicates": status.duplicates,
        "gaps": status.gaps,
        "lostCycles": status.lost_cycles,
        "droppedIncomplete": status.dropped_incomplete,
        "backfills": status.backfills,
        "restarts": status.restarts,
        "malformedDatagrams": status.malformed_datagrams,
        "server": server.map(|s| json!({
            "clients": s.clients,
            "bytesPerSecond": s.bytes_per_second,
            "budgetBytesPerSecond": s.budget_bytes_per_second,
            "throttled": s.throttled,
            "throttledPatterns": s.throttled_patterns,
            "droppedFrames": s.dropped_frames,
        })),
    })
}

#[tauri::command]
pub async fn live_status(runtime: State<'_, LiveRuntime>) -> Result<serde_json::Value, String> {
    let c = client(&runtime)?;
    Ok(status_json(&c.status(), c.server_status().as_ref()))
}

/// Drains the client's events on its own thread. Records are batched; status and link events are
/// emitted directly. Ends after the client reports disconnection.
fn run_pump(app: AppHandle, rx: Receiver<Event>, client: Client, recorder: Arc<Mutex<Option<Recorder>>>) {
    let mut batch: Vec<Record> = Vec::new();
    let mut frames = 0u64;
    let mut last_flush = Instant::now();
    let mut last_status = Instant::now() - STATUS_INTERVAL;
    loop {
        let mut disconnected = None;
        match rx.recv_timeout(BATCH_INTERVAL) {
            Ok(event) => disconnected = absorb(&app, event, &mut batch, &mut frames),
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => disconnected = Some("client closed".to_owned()),
        }
        if last_flush.elapsed() >= BATCH_INTERVAL || disconnected.is_some() {
            flush(&app, &mut batch, &mut frames, &recorder);
            last_flush = Instant::now();
        }
        if last_status.elapsed() >= STATUS_INTERVAL || disconnected.is_some() {
            let status = client.status();
            let server = client.server_status();
            let _ = app.emit("live://status", status_json(&status, server.as_ref()));
            last_status = Instant::now();
        }
        if let Some(reason) = disconnected {
            let _ = app.emit("live://event", json!({ "kind": "disconnected", "reason": reason }));
            break;
        }
    }
}

/// Queues one event for the next batch. Returns the reason when the connection has ended.
fn absorb(app: &AppHandle, event: Event, batch: &mut Vec<Record>, frames: &mut u64) -> Option<String> {
    match event {
        Event::Dictionary { dictionary, schemas } => {
            batch.push(Record::Dictionary(dictionary));
            batch.extend(schemas.into_iter().map(Record::StructSchema));
        }
        Event::Frame(frame) => {
            *frames += 1;
            batch.push(Record::Frame(frame));
        }
        Event::Gap { from, to, lost } => {
            let _ = app.emit("live://event", json!({ "kind": "gap", "from": from, "to": to, "lost": lost }));
        }
        Event::Backfill { from, to, result } => match result {
            Ok(records) => {
                let _ = app.emit(
                    "live://event",
                    json!({ "kind": "backfill", "from": from, "to": to, "records": records.len() }),
                );
                batch.extend(records);
            }
            Err(error) => {
                let _ = app.emit(
                    "live://event",
                    json!({ "kind": "backfill", "from": from, "to": to, "error": error }),
                );
            }
        },
        Event::Record(record) => batch.push(record),
        Event::Disconnected { reason } => return Some(reason),
    }
    None
}

fn flush(app: &AppHandle, batch: &mut Vec<Record>, frames: &mut u64, recorder: &Arc<Mutex<Option<Recorder>>>) {
    if batch.is_empty() {
        return;
    }
    let mut w = Writer::new();
    for record in batch.iter() {
        record.encode(&mut w);
    }
    let data = base64::engine::general_purpose::STANDARD.encode(w.as_slice());
    let _ = app.emit("live://records", json!({ "data": data, "frames": *frames }));
    if let Some(rec) = recorder.lock().unwrap_or_else(|p| p.into_inner()).as_mut() {
        if rec.append(batch).is_err() {
            // The file is unusable (disk full, removed). Stop recording and tell the UI.
            let _ = app.emit("live://event", json!({ "kind": "recordingFailed" }));
        }
    }
    batch.clear();
    *frames = 0;
}
