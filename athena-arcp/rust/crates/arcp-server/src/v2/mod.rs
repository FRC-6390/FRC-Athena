//! ARCP v2 server (E1-E6).
//!
//! * Realtime: one batched frame per robot cycle over UDP. Each subscriber receives only the
//!   signals its subscriptions match, at the requested rate, with per-subscriber sequence numbers
//!   and keyframes at least once per [`ServerConfig::keyframe_interval`].
//! * Control: reliable TCP with typed binary requests, request ids and structured replies.
//! * Writes and actions are forwarded to the robot through [`RobotHandler`]; the robot validates
//!   and the reply carries its accept or reject reason.
//!
//! The server runs on its own threads. Calls on [`ArcpServerV2`] are cheap and thread safe; the
//! robot loop should call [`ArcpServerV2::publish_cycle`] once per cycle.

mod conn;
mod files;
mod state;

use std::io;
use std::net::{SocketAddr, TcpListener, UdpSocket};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use arcp_core::v2::control::Rate;
use arcp_core::v2::record::{Dictionary, Marker, StructSchema};
use arcp_core::v2::{Entry, Value};

use state::State;

pub use arcp_core::v2::control::{WriteReply, MAX_FILE_CHUNK};

/// Robot-side hook. Called on control connection threads, so implementations should hand the
/// request to the robot loop and wait for its decision (or return quickly).
pub trait RobotHandler: Send + Sync + 'static {
    /// Validates and applies a write. The returned [`WriteReply`] (accepted or rejected with a
    /// reason, plus the value the robot now holds) is sent to the client unchanged.
    fn write(&self, signal_id: u64, value: Value) -> WriteReply;
    /// Runs (`run == true`) or cancels (`run == false`) the Action behind `signal_id`.
    fn action(&self, signal_id: u64, run: bool) -> Result<(), String>;
    /// Returns buffered replay record groups covering frame sequences `from..=to` (see
    /// [`ReplyData::Records`]). The default has no buffer.
    fn backfill(&self, _from: u64, _to: u64) -> Result<Vec<u8>, String> {
        Err("backfill not supported".into())
    }
}

#[derive(Debug, Clone)]
pub struct ServerConfig {
    /// TCP control listener address. Port 0 picks a free port; see [`ArcpServerV2::control_addr`].
    /// Default port is [`DEFAULT_CONTROL_PORT`] (5808).
    pub control_bind: SocketAddr,
    /// UDP port the realtime frames are sent from (bound on all interfaces). Port 0 picks a free
    /// port; see [`ArcpServerV2::realtime_addr`]. Default is [`DEFAULT_REALTIME_PORT`] (5809).
    pub realtime_port: u16,
    /// Directory exposed by the file service for plain names. `None` disables plain names.
    pub file_root: Option<PathBuf>,
    /// Named file-service categories (`category/name`), each a directory. Missing directories
    /// list as empty.
    pub file_categories: Vec<(String, PathBuf)>,
    /// Realtime bandwidth budget in bytes per second. `0` disables budgeting.
    pub bandwidth_budget_bytes_per_sec: u64,
    /// Maximum datagram size for frames (fragments split at this size).
    pub mtu: usize,
    /// Keyframe cadence per subscriber.
    pub keyframe_interval: Duration,
}

/// Default TCP control port. Inside the FRC team-use range 5800-5810 (NT4 uses 5810).
pub const DEFAULT_CONTROL_PORT: u16 = 5808;
/// Default UDP port for realtime frames, inside the FRC team-use range.
pub const DEFAULT_REALTIME_PORT: u16 = 5809;

impl ServerConfig {
    /// The default configuration with both sockets on ephemeral ports (for tests and tools that
    /// run alongside other servers).
    pub fn ephemeral() -> Self {
        Self { control_bind: SocketAddr::from(([0, 0, 0, 0], 0)), realtime_port: 0, ..Self::default() }
    }
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            control_bind: SocketAddr::from(([0, 0, 0, 0], DEFAULT_CONTROL_PORT)),
            realtime_port: DEFAULT_REALTIME_PORT,
            file_root: None,
            file_categories: Vec::new(),
            bandwidth_budget_bytes_per_sec: 0,
            mtu: 1200,
            keyframe_interval: Duration::from_secs(1),
        }
    }
}

/// A subscription whose effective rate was lowered by the bandwidth budget.
#[derive(Debug, Clone, PartialEq)]
pub struct ThrottledSubscription {
    pub session_id: u32,
    pub subscription_id: u32,
    pub pattern: String,
    pub requested: Rate,
    pub divisor: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ServerStatus {
    pub sessions: usize,
    pub subscriptions: usize,
    pub throttled: Vec<ThrottledSubscription>,
    /// Realtime bytes sent in the last completed budget window.
    pub bytes_last_window: u64,
}

pub(crate) struct Inner {
    state: Mutex<State>,
    pub(crate) udp: UdpSocket,
    pub(crate) handler: Arc<dyn RobotHandler>,
    pub(crate) roots: files::Roots,
    started: Instant,
    shutdown: AtomicBool,
}

impl Inner {
    pub(crate) fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    pub(crate) fn elapsed_micros(&self) -> u64 {
        self.started.elapsed().as_micros() as u64
    }
}

/// Handle to a running v2 server. Cloning shares the same server.
#[derive(Clone)]
pub struct ArcpServerV2 {
    inner: Arc<Inner>,
    control_addr: SocketAddr,
    realtime_addr: SocketAddr,
}

impl ArcpServerV2 {
    /// Binds the control listener and starts the accept thread.
    pub fn start(config: ServerConfig, handler: Arc<dyn RobotHandler>) -> io::Result<Self> {
        let listener = TcpListener::bind(config.control_bind)?;
        listener.set_nonblocking(true)?;
        let control_addr = listener.local_addr()?;
        let udp = UdpSocket::bind(SocketAddr::from(([0, 0, 0, 0], config.realtime_port)))?;
        let realtime_addr = udp.local_addr()?;
        let keyframe_micros = config.keyframe_interval.as_micros().min(u128::from(u64::MAX)) as u64;
        let inner = Arc::new(Inner {
            state: Mutex::new(State::new(config.bandwidth_budget_bytes_per_sec, config.mtu, keyframe_micros)),
            udp,
            handler,
            roots: files::Roots { root: config.file_root, categories: config.file_categories },
            started: Instant::now(),
            shutdown: AtomicBool::new(false),
        });
        let accept_inner = Arc::clone(&inner);
        thread::Builder::new()
            .name("arcp-v2-accept".into())
            .spawn(move || accept_loop(accept_inner, listener))?;
        Ok(Self { inner, control_addr, realtime_addr })
    }

    /// Address of the TCP control listener (useful when the configured port was 0).
    pub fn control_addr(&self) -> SocketAddr {
        self.control_addr
    }

    /// Address the realtime UDP socket is bound to (useful when the configured port was 0).
    pub fn realtime_addr(&self) -> SocketAddr {
        self.realtime_addr
    }

    /// Replaces the dictionary and struct schemas, pushes them to every connected client, and
    /// re-matches existing subscriptions.
    pub fn publish_dictionary(&self, dictionary: Dictionary, schemas: Vec<StructSchema>) {
        self.inner.lock().set_dictionary(dictionary, schemas);
    }

    /// Publishes one robot cycle. `values` holds the cycle's current signal values; unchanged
    /// values are ignored, and each subscriber receives one batched frame with what is due.
    pub fn publish_cycle(&self, cycle_timestamp_micros: u64, values: &[Entry]) {
        self.inner.lock().publish_cycle(cycle_timestamp_micros, values, &self.inner.udp);
    }

    /// Sends a marker to every subscriber (markers are not filtered by subscriptions).
    pub fn marker(&self, marker: Marker) {
        self.inner.lock().send_marker(marker, &self.inner.udp);
    }

    /// Sends raw replay records (a concatenation of complete record envelopes) to every session that
    /// subscribed with [`ClientMessage::ReplaySubscribe`]. Records are packed into datagrams of at
    /// most `mtu` bytes; a single larger record is sent alone.
    pub fn publish_records(&self, records: &[u8]) {
        self.inner.lock().publish_records(records, &self.inner.udp);
    }

    /// Changes whenever the set of subscribed signals may have changed. A caller that caches
    /// [`Self::subscribed_signal_ids`] compares this value first and re-reads the ids only when it differs.
    pub fn subscription_generation(&self) -> u64 {
        self.inner.lock().subscription_generation
    }

    /// Sorted ids of the signals that at least one subscription of any session matches.
    pub fn subscribed_signal_ids(&self) -> Vec<u64> {
        self.inner.lock().subscribed_ids()
    }

    /// Number of connected control sessions.
    pub fn session_count(&self) -> usize {
        self.inner.lock().sessions.len()
    }

    pub fn status(&self) -> ServerStatus {
        let state = self.inner.lock();
        let mut throttled = Vec::new();
        let mut subscriptions = 0;
        for session in &state.sessions {
            for sub in &session.subs {
                subscriptions += 1;
                if sub.divisor > 1 {
                    throttled.push(ThrottledSubscription {
                        session_id: session.id,
                        subscription_id: sub.id,
                        pattern: sub.pattern.clone(),
                        requested: sub.rate,
                        divisor: sub.divisor,
                    });
                }
            }
        }
        ServerStatus {
            sessions: state.sessions.len(),
            subscriptions,
            throttled,
            bytes_last_window: state.last_window_bytes,
        }
    }

    /// Stops accepting new connections. Existing connections end when their clients close them.
    pub fn shutdown(&self) {
        self.inner.shutdown.store(true, Ordering::SeqCst);
    }
}

fn accept_loop(inner: Arc<Inner>, listener: TcpListener) {
    while !inner.shutdown.load(Ordering::SeqCst) {
        match listener.accept() {
            Ok((stream, _)) => {
                if stream.set_nonblocking(false).is_err() {
                    continue;
                }
                let conn_inner = Arc::clone(&inner);
                let _ = thread::Builder::new()
                    .name("arcp-v2-conn".into())
                    .spawn(move || conn::serve(conn_inner, stream));
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => thread::sleep(Duration::from_millis(5)),
            Err(_) => thread::sleep(Duration::from_millis(50)),
        }
    }
}
