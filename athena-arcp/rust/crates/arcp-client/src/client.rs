//! Public client: connect, request/reply commands, time sync, file service, status.

use std::collections::HashMap;
use std::fmt;
use std::io::Write;
use std::net::{Shutdown, SocketAddr, TcpStream, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use arcp_core::v2::control::{
    encode_client, frame, ClientMessage, FileInfo, Rate, ReplyData, ServerMessage, Status, WriteReply,
    CONTROL_PROTOCOL_VERSION, MAX_FILE_CHUNK,
};
use arcp_core::v2::record::{decode_stream, Dictionary, Record, StructSchema};
use arcp_core::v2::wire::{Reader, WireError};
use arcp_core::v2::Value;

use crate::model::Model;
use crate::timesync::{estimate, TimeEstimate, TimeSample};

pub const DEFAULT_REQUEST_TIMEOUT: Duration = Duration::from_secs(2);
/// Default TCP control port of the robot (inside the FRC team range 5800-5810; NT4 uses 5810).
pub const DEFAULT_CONTROL_PORT: u16 = 5808;
/// Default local UDP port for realtime frames. If it is taken, the client falls back to an
/// ephemeral port and reports it through [`Client::udp_fallback`].
pub const DEFAULT_UDP_PORT: u16 = 5807;

/// Connection settings.
#[derive(Debug, Clone)]
pub struct ClientConfig {
    /// Robot or simulator host (`10.TE.AM.2`, `localhost`, ...).
    pub host: String,
    pub control_port: u16,
    pub client_name: String,
    pub request_timeout: Duration,
    /// Local UDP address for realtime frames (default port 5807). If this port cannot be bound
    /// the client binds an ephemeral port instead and sets [`Client::udp_fallback`]. Port 0 picks one.
    pub udp_bind: SocketAddr,
    /// Port advertised to the robot in `Hello`. `None` advertises the bound port. Set this when
    /// the path to the client is port-forwarded or proxied.
    pub udp_advertise: Option<u16>,
    /// Subscribe to raw replay records (`ReplaySubscribe`) after connecting.
    pub replay: bool,
    /// Request buffered records for every frame sequence gap.
    pub auto_backfill: bool,
    /// Subscription made right after connecting (pattern, rate). The default is every signal on
    /// change (plus keyframes), so a dashboard gets values without asking; `None` subscribes to nothing.
    pub initial_subscription: Option<(String, Rate)>,
}

impl ClientConfig {
    pub fn new(host: impl Into<String>, control_port: u16) -> Self {
        Self {
            host: host.into(),
            control_port,
            client_name: "athena-dashboard".into(),
            request_timeout: DEFAULT_REQUEST_TIMEOUT,
            udp_bind: SocketAddr::from(([0, 0, 0, 0], DEFAULT_UDP_PORT)),
            udp_advertise: None,
            replay: false,
            auto_backfill: true,
            initial_subscription: Some(("**".into(), Rate::OnChange)),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ClientError {
    Io(String),
    Timeout,
    Disconnected,
    Protocol(String),
    /// The robot or server answered with a non-OK status.
    Server { status: Status, reason: String },
}

impl fmt::Display for ClientError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ClientError::Io(e) => write!(f, "I/O error: {e}"),
            ClientError::Timeout => write!(f, "request timed out"),
            ClientError::Disconnected => write!(f, "disconnected from robot"),
            ClientError::Protocol(e) => write!(f, "protocol error: {e}"),
            ClientError::Server { status, reason } => write!(f, "{status:?}: {reason}"),
        }
    }
}

impl std::error::Error for ClientError {}

/// Things the client observed, delivered in order over the channel returned by
/// [`Client::connect`].
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// Full dictionary and schemas: on connect, on a server push, and after `SchemaRequest`.
    Dictionary { dictionary: Dictionary, schemas: Vec<StructSchema> },
    /// A complete, deduplicated frame (one robot cycle).
    Frame(Frame),
    /// Frame sequences `from..=to` (inclusive) were lost on the way.
    Gap { from: u32, to: u32, lost: u32 },
    /// Result of an automatic backfill for a gap.
    Backfill { from: u64, to: u64, result: Result<Vec<Record>, String> },
    /// Non-frame records that arrive over UDP once replay is subscribed (markers, metadata).
    Record(Record),
    /// The control connection ended. No further events follow.
    Disconnected { reason: String },
}

use arcp_core::v2::record::Frame;

/// Snapshot of link health for the status bar.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinkStatus {
    pub connected: bool,
    pub session_id: u32,
    /// From the most recent time sync.
    pub rtt_micros: Option<f64>,
    pub offset_micros: Option<f64>,
    pub frames: u64,
    pub duplicates: u64,
    pub gaps: u64,
    /// Frame cycles lost to sequence gaps.
    pub lost_cycles: u64,
    /// Cycles abandoned because a fragment never arrived.
    pub dropped_incomplete: u64,
    pub backfills: u64,
    pub restarts: u64,
    pub malformed_datagrams: u64,
    /// The configured realtime UDP port was taken; an ephemeral port is receiving frames.
    pub udp_fallback: bool,
}

#[derive(Default)]
pub(crate) struct Counters {
    pub frames: AtomicU64,
    pub duplicates: AtomicU64,
    pub gaps: AtomicU64,
    pub lost_cycles: AtomicU64,
    pub dropped_incomplete: AtomicU64,
    pub backfills: AtomicU64,
    pub restarts: AtomicU64,
    pub malformed: AtomicU64,
}

pub(crate) struct Shared {
    pub model: Mutex<Model>,
    pub pending: Mutex<HashMap<u32, Sender<ServerMessage>>>,
    pub writer: Mutex<TcpStream>,
    pub events: Sender<Event>,
    pub connected: AtomicBool,
    pub shutdown: AtomicBool,
    pub session_id: AtomicU32,
    pub next_id: AtomicU32,
    pub request_timeout: Duration,
    pub auto_backfill: bool,
    pub counters: Counters,
    pub time: Mutex<Option<TimeEstimate>>,
    pub start: Instant,
}

pub(crate) fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

impl Shared {
    fn now_micros(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }

    fn next_request_id(&self) -> u32 {
        loop {
            let id = self.next_id.fetch_add(1, Ordering::SeqCst);
            if id != 0 {
                return id;
            }
        }
    }
}

/// Handle to a connected robot. Cheap to clone; all clones share one connection.
#[derive(Clone)]
pub struct Client {
    shared: Arc<Shared>,
    udp_port: u16,
    udp_fallback: bool,
    control_addr: SocketAddr,
}

impl Client {
    /// Opens the control and realtime sockets, runs the Hello exchange and subscribes (see
    /// [`ClientConfig::initial_subscription`]) and optionally to replay records. Events arrive on the returned receiver.
    pub fn connect(config: ClientConfig) -> Result<(Client, Receiver<Event>), ClientError> {
        let addr = (config.host.as_str(), config.control_port)
            .to_socket_addrs()
            .map_err(|e| ClientError::Io(e.to_string()))?
            .next()
            .ok_or_else(|| ClientError::Io(format!("cannot resolve {}", config.host)))?;
        let stream = TcpStream::connect_timeout(&addr, config.request_timeout)
            .map_err(|e| ClientError::Io(format!("connect {addr}: {e}")))?;
        let _ = stream.set_nodelay(true);
        let _ = stream.set_write_timeout(Some(config.request_timeout));
        let reader = stream.try_clone().map_err(|e| ClientError::Io(e.to_string()))?;

        let (udp, udp_fallback) = match UdpSocket::bind(config.udp_bind) {
            Ok(socket) => (socket, false),
            Err(_) if config.udp_bind.port() != 0 => {
                let fallback = SocketAddr::new(config.udp_bind.ip(), 0);
                let socket = UdpSocket::bind(fallback)
                    .map_err(|e| ClientError::Io(format!("udp bind: {e}")))?;
                (socket, true)
            }
            Err(e) => return Err(ClientError::Io(format!("udp bind: {e}"))),
        };
        udp.set_read_timeout(Some(Duration::from_millis(200)))
            .map_err(|e| ClientError::Io(e.to_string()))?;
        let udp_port = udp.local_addr().map_err(|e| ClientError::Io(e.to_string()))?.port();

        let (tx, rx) = mpsc::channel();
        let shared = Arc::new(Shared {
            model: Mutex::new(Model::default()),
            pending: Mutex::new(HashMap::new()),
            writer: Mutex::new(stream),
            events: tx,
            connected: AtomicBool::new(true),
            shutdown: AtomicBool::new(false),
            session_id: AtomicU32::new(0),
            next_id: AtomicU32::new(1),
            request_timeout: config.request_timeout,
            auto_backfill: config.auto_backfill,
            counters: Counters::default(),
            time: Mutex::new(None),
            start: Instant::now(),
        });

        let control_shared = Arc::clone(&shared);
        thread::Builder::new()
            .name("arcp-client-control".into())
            .spawn(move || crate::control::run_reader(control_shared, reader))
            .map_err(|e| ClientError::Io(e.to_string()))?;
        let udp_shared = Arc::clone(&shared);
        thread::Builder::new()
            .name("arcp-client-udp".into())
            .spawn(move || crate::udp::run(udp_shared, udp))
            .map_err(|e| ClientError::Io(e.to_string()))?;

        let client = Client { shared, udp_port, udp_fallback, control_addr: addr };
        let advertised = config.udp_advertise.unwrap_or(udp_port);
        let hello = ClientMessage::Hello {
            client_name: config.client_name.clone(),
            protocol_version: CONTROL_PROTOCOL_VERSION,
            udp_port: advertised,
        };
        match client.reply(hello) {
            Ok((Status::Ok, _, ReplyData::Session { session_id })) => {
                client.shared.session_id.store(session_id, Ordering::SeqCst);
            }
            Ok((status, reason, _)) => {
                client.close();
                return Err(ClientError::Server { status, reason });
            }
            Err(e) => {
                client.close();
                return Err(e);
            }
        }
        if config.replay {
            if let Err(e) = client.replay_subscribe() {
                client.close();
                return Err(e);
            }
        }
        if let Some((pattern, rate)) = &config.initial_subscription {
            if let Err(e) = client.subscribe(pattern, *rate) {
                client.close();
                return Err(e);
            }
        }
        Ok((client, rx))
    }

    /// Closes both sockets. The event channel then ends with `Disconnected`.
    pub fn close(&self) {
        self.shared.shutdown.store(true, Ordering::SeqCst);
        let _ = lock(&self.shared.writer).shutdown(Shutdown::Both);
    }

    pub fn control_addr(&self) -> SocketAddr {
        self.control_addr
    }

    pub fn udp_port(&self) -> u16 {
        self.udp_port
    }

    /// True when the configured realtime UDP port was taken and an ephemeral port is in use.
    pub fn udp_fallback(&self) -> bool {
        self.udp_fallback
    }

    pub fn is_connected(&self) -> bool {
        self.shared.connected.load(Ordering::SeqCst)
    }

    /// Creates a subscription. The robot replies with its id; matching latest values arrive as
    /// the next frame.
    pub fn subscribe(&self, pattern: &str, rate: Rate) -> Result<u32, ClientError> {
        let (status, reason, data) = self.reply(ClientMessage::Subscribe { pattern: pattern.into(), rate })?;
        check(status, reason)?;
        match data {
            ReplyData::Subscription { subscription_id } => Ok(subscription_id),
            other => Err(ClientError::Protocol(format!("subscribe reply data {other:?}"))),
        }
    }

    pub fn unsubscribe(&self, subscription_id: u32) -> Result<(), ClientError> {
        let (status, reason, _) = self.reply(ClientMessage::Unsubscribe { subscription_id })?;
        check(status, reason)
    }

    /// Sends a write. The robot decides; a rejection is an `Ok` reply with `accepted == false`,
    /// the reason, and the value the robot still holds. Nothing is blocked on the client (E5).
    pub fn write(&self, signal_id: u64, value: Value) -> Result<WriteReply, ClientError> {
        match self.request(ClientMessage::Write { signal_id, value })? {
            ServerMessage::WriteReply(reply) => {
                if reply.accepted {
                    lock(&self.shared.model).set_value(signal_id, reply.actual_value.clone());
                }
                Ok(reply)
            }
            ServerMessage::Reply { status, reason, .. } => Err(ClientError::Server { status, reason }),
            other => Err(ClientError::Protocol(format!("write reply {other:?}"))),
        }
    }

    pub fn action_run(&self, signal_id: u64) -> Result<(), ClientError> {
        let (status, reason, _) = self.reply(ClientMessage::ActionRun { signal_id })?;
        check(status, reason)
    }

    pub fn action_cancel(&self, signal_id: u64) -> Result<(), ClientError> {
        let (status, reason, _) = self.reply(ClientMessage::ActionCancel { signal_id })?;
        check(status, reason)
    }

    /// Runs `samples` ping/pong exchanges and keeps the lowest-RTT estimate.
    pub fn time_sync(&self, samples: usize) -> Result<TimeEstimate, ClientError> {
        let mut collected = Vec::with_capacity(samples.max(1));
        for _ in 0..samples.max(1) {
            let t0 = self.shared.now_micros();
            match self.request(ClientMessage::TimePing { client_micros: t0 })? {
                ServerMessage::TimePong { client_micros, server_micros } => collected.push(TimeSample {
                    client_send_micros: client_micros,
                    server_micros,
                    client_recv_micros: self.shared.now_micros(),
                }),
                other => return Err(ClientError::Protocol(format!("time reply {other:?}"))),
            }
        }
        let est = estimate(&collected).ok_or_else(|| ClientError::Protocol("no usable time samples".into()))?;
        *lock(&self.shared.time) = Some(est);
        Ok(est)
    }

    /// Asks for the dictionary and schemas again. The answer arrives as [`Event::Dictionary`].
    pub fn schema_request(&self) -> Result<(), ClientError> {
        match self.request(ClientMessage::SchemaRequest)? {
            ServerMessage::Schema { .. } => Ok(()),
            ServerMessage::Reply { status, reason, .. } => Err(ClientError::Server { status, reason }),
            other => Err(ClientError::Protocol(format!("schema reply {other:?}"))),
        }
    }

    pub fn replay_subscribe(&self) -> Result<(), ClientError> {
        let (status, reason, _) = self.reply(ClientMessage::ReplaySubscribe)?;
        check(status, reason)
    }

    /// Returns buffered records for frame sequences `from..=to`. Each group decodes on its own.
    pub fn backfill(&self, from: u64, to: u64) -> Result<Vec<Record>, ClientError> {
        backfill_records(&self.shared, from, to)
    }

    pub fn file_list(&self) -> Result<Vec<FileInfo>, ClientError> {
        let (status, reason, data) = self.reply(ClientMessage::FileList)?;
        check(status, reason)?;
        match data {
            ReplyData::Files(files) => Ok(files),
            other => Err(ClientError::Protocol(format!("file list data {other:?}"))),
        }
    }

    /// Downloads a whole file in chunks. Fails instead of returning more than `max_bytes`.
    pub fn file_download(&self, name: &str, max_bytes: usize) -> Result<Vec<u8>, ClientError> {
        let mut out = Vec::new();
        loop {
            let (status, reason, data) = self.reply(ClientMessage::FileDownload {
                name: name.into(),
                offset: out.len() as u64,
                max_len: MAX_FILE_CHUNK,
            })?;
            check(status, reason)?;
            let ReplyData::Chunk { eof, data } = data else {
                return Err(ClientError::Protocol("download reply is not a chunk".into()));
            };
            if out.len() + data.len() > max_bytes {
                return Err(ClientError::Protocol(format!("{name} is larger than {max_bytes} bytes")));
            }
            if data.is_empty() && !eof {
                return Err(ClientError::Protocol("download stalled".into()));
            }
            out.extend_from_slice(&data);
            if eof {
                return Ok(out);
            }
        }
    }

    pub fn file_delete(&self, name: &str) -> Result<(), ClientError> {
        let (status, reason, _) = self.reply(ClientMessage::FileDelete { name: name.into() })?;
        check(status, reason)
    }

    pub fn dictionary(&self) -> Dictionary {
        lock(&self.shared.model).dictionary().clone()
    }

    pub fn schemas(&self) -> Vec<StructSchema> {
        lock(&self.shared.model).schemas().to_vec()
    }

    pub fn signal_id(&self, path: &str) -> Option<u64> {
        lock(&self.shared.model).signal_id(path)
    }

    pub fn value(&self, signal_id: u64) -> Option<Value> {
        lock(&self.shared.model).value(signal_id).cloned()
    }

    pub fn lock_state(&self, group: &str) -> Option<bool> {
        lock(&self.shared.model).lock_state(group)
    }

    pub fn server_status(&self) -> Option<crate::model::ServerStatus> {
        lock(&self.shared.model).server_status()
    }

    pub fn status(&self) -> LinkStatus {
        let c = &self.shared.counters;
        let time = *lock(&self.shared.time);
        LinkStatus {
            connected: self.is_connected(),
            session_id: self.shared.session_id.load(Ordering::SeqCst),
            rtt_micros: time.map(|t| t.rtt_micros),
            offset_micros: time.map(|t| t.offset_micros),
            frames: c.frames.load(Ordering::Relaxed),
            duplicates: c.duplicates.load(Ordering::Relaxed),
            gaps: c.gaps.load(Ordering::Relaxed),
            lost_cycles: c.lost_cycles.load(Ordering::Relaxed),
            dropped_incomplete: c.dropped_incomplete.load(Ordering::Relaxed),
            backfills: c.backfills.load(Ordering::Relaxed),
            restarts: c.restarts.load(Ordering::Relaxed),
            malformed_datagrams: c.malformed.load(Ordering::Relaxed),
            udp_fallback: self.udp_fallback,
        }
    }

    fn request(&self, message: ClientMessage) -> Result<ServerMessage, ClientError> {
        request(&self.shared, message)
    }

    fn reply(&self, message: ClientMessage) -> Result<(Status, String, ReplyData), ClientError> {
        match self.request(message)? {
            ServerMessage::Reply { status, reason, data } => Ok((status, reason, data)),
            other => Err(ClientError::Protocol(format!("expected reply, got {other:?}"))),
        }
    }
}

fn check(status: Status, reason: String) -> Result<(), ClientError> {
    match status {
        Status::Ok => Ok(()),
        _ => Err(ClientError::Server { status, reason }),
    }
}

fn request(shared: &Shared, message: ClientMessage) -> Result<ServerMessage, ClientError> {
    if !shared.connected.load(Ordering::SeqCst) {
        return Err(ClientError::Disconnected);
    }
    let id = shared.next_request_id();
    let (tx, rx) = mpsc::channel();
    lock(&shared.pending).insert(id, tx);
    let bytes = match frame(&encode_client(id, &message)) {
        Ok(bytes) => bytes,
        Err(e) => {
            lock(&shared.pending).remove(&id);
            return Err(ClientError::Protocol(e.to_string()));
        }
    };
    let written = {
        let mut writer = lock(&shared.writer);
        writer.write_all(&bytes).and_then(|()| writer.flush())
    };
    if let Err(e) = written {
        lock(&shared.pending).remove(&id);
        return Err(ClientError::Io(e.to_string()));
    }
    match rx.recv_timeout(shared.request_timeout) {
        Ok(reply) => Ok(reply),
        Err(RecvTimeoutError::Timeout) => {
            lock(&shared.pending).remove(&id);
            Err(ClientError::Timeout)
        }
        Err(RecvTimeoutError::Disconnected) => Err(ClientError::Disconnected),
    }
}

pub(crate) fn backfill_records(shared: &Shared, from: u64, to: u64) -> Result<Vec<Record>, ClientError> {
    match request(shared, ClientMessage::Backfill { from, to })? {
        ServerMessage::Reply { status: Status::Ok, data: ReplyData::Records(data), .. } => {
            decode_backfill(&data).map_err(|e| ClientError::Protocol(e.to_string()))
        }
        ServerMessage::Reply { status, reason, .. } => Err(ClientError::Server { status, reason }),
        other => Err(ClientError::Protocol(format!("backfill reply {other:?}"))),
    }
}

/// Decodes a backfill payload: repeated `varint groupLength, group` where each group is a record
/// stream (appendix E1-W).
pub fn decode_backfill(data: &[u8]) -> Result<Vec<Record>, WireError> {
    let mut reader = Reader::new(data);
    let mut out = Vec::new();
    while reader.has_remaining() {
        let len = reader.length()?;
        let group = reader.bytes(len)?;
        out.extend(decode_stream(group)?);
    }
    Ok(out)
}
