//! Shared server state, per-subscriber pacing and realtime frame building (E1, E2).

use std::collections::{BTreeMap, HashMap};
use std::io::Write;
use std::net::{SocketAddr, TcpStream, UdpSocket};
use std::sync::{Arc, Mutex};

use arcp_core::v2::control::{encode_server, frame, Rate, ServerMessage};
use arcp_core::v2::pattern;
use arcp_core::v2::record::{Dictionary, Marker, Record, StructSchema, FRAME_FLAG_FRAGMENT, FRAME_FLAG_KEYFRAME, FRAME_VERSION, RECORD_FRAME};
use arcp_core::v2::{Entry, Writer};

/// Largest number of times a subscription's rate may be divided by the budget logic.
const MAX_DIVISOR: u32 = 16;
/// Bytes reserved per datagram for the record envelope and frame header.
const FRAME_OVERHEAD: usize = 64;

/// Last value sent to one subscriber for one signal.
pub(crate) struct Sent {
    pub at: u64,
    pub value: arcp_core::v2::Value,
}

pub(crate) struct Subscription {
    pub id: u32,
    pub pattern: String,
    pub rate: Rate,
    /// Budget throttle: effective rate is `rate / divisor`.
    pub divisor: u32,
    /// Signal ids whose dictionary path matches `pattern`.
    pub matches: Vec<u64>,
}

impl Subscription {
    fn effective(&self) -> Policy {
        match self.rate {
            Rate::OnChange => Policy::OnChange,
            Rate::Hz(hz) => Policy::Hz(hz / f64::from(self.divisor)),
        }
    }
}

#[derive(Clone, Copy)]
enum Policy {
    OnChange,
    Hz(f64),
}

impl Policy {
    /// Highest rate wins when several subscriptions cover one signal.
    fn merge(self, other: Policy) -> Policy {
        match (self, other) {
            (Policy::Hz(a), Policy::Hz(b)) => Policy::Hz(a.max(b)),
            (Policy::Hz(a), Policy::OnChange) | (Policy::OnChange, Policy::Hz(a)) => Policy::Hz(a),
            (Policy::OnChange, Policy::OnChange) => Policy::OnChange,
        }
    }
}

pub(crate) struct Session {
    pub id: u32,
    pub name: String,
    pub udp: Option<SocketAddr>,
    /// Shared with the connection thread so replies can be written without the state lock.
    pub writer: Arc<Mutex<TcpStream>>,
    pub subs: Vec<Subscription>,
    pub next_sub_id: u32,
    pub sent: HashMap<u64, Sent>,
    pub seq: u32,
    pub last_keyframe: Option<u64>,
    pub dead: bool,
}

pub(crate) struct State {
    pub dictionary: Dictionary,
    pub schemas: Vec<StructSchema>,
    pub latest: HashMap<u64, arcp_core::v2::Value>,
    pub latest_ts: u64,
    pub sessions: Vec<Session>,
    pub next_session_id: u32,
    pub budget_bytes_per_sec: u64,
    pub mtu: usize,
    pub keyframe_interval_micros: u64,
    pub window_start: u64,
    pub window_bytes: u64,
    pub last_window_bytes: u64,
}

impl State {
    pub fn new(budget_bytes_per_sec: u64, mtu: usize, keyframe_interval_micros: u64) -> Self {
        Self {
            dictionary: Dictionary::default(),
            schemas: Vec::new(),
            latest: HashMap::new(),
            latest_ts: 0,
            sessions: Vec::new(),
            next_session_id: 1,
            budget_bytes_per_sec,
            mtu: mtu.max(256),
            keyframe_interval_micros,
            window_start: 0,
            window_bytes: 0,
            last_window_bytes: 0,
        }
    }

    pub fn add_session(&mut self, writer: Arc<Mutex<TcpStream>>) -> u32 {
        let id = self.next_session_id;
        self.next_session_id += 1;
        self.sessions.push(Session {
            id,
            name: String::new(),
            udp: None,
            writer,
            subs: Vec::new(),
            next_sub_id: 1,
            sent: HashMap::new(),
            seq: 0,
            last_keyframe: None,
            dead: false,
        });
        id
    }

    pub fn session_mut(&mut self, id: u32) -> Option<&mut Session> {
        self.sessions.iter_mut().find(|s| s.id == id)
    }

    pub fn remove_session(&mut self, id: u32) {
        self.sessions.retain(|s| s.id != id);
    }

    /// Pushes the full schema to every connected client (request id 0).
    pub fn push_schema(&mut self) {
        let message = self.schema_message();
        for session in &mut self.sessions {
            if !send(&session.writer, 0, &message) {
                session.dead = true;
            }
        }
    }

    pub fn schema_message(&self) -> ServerMessage {
        ServerMessage::Schema { dictionary: self.dictionary.clone(), schemas: self.schemas.clone() }
    }

    /// Replaces the dictionary and recomputes every subscription's matches.
    pub fn set_dictionary(&mut self, dictionary: Dictionary, schemas: Vec<StructSchema>) {
        self.dictionary = dictionary;
        self.schemas = schemas;
        let dict = &self.dictionary;
        for session in &mut self.sessions {
            for sub in &mut session.subs {
                sub.matches = match_signals(dict, &sub.pattern);
            }
        }
        self.push_schema();
        self.sessions.retain(|s| !s.dead);
    }

    /// Records the cycle's values (only real changes are stored) and sends every due update.
    pub fn publish_cycle(&mut self, ts: u64, values: &[Entry], udp: &UdpSocket) {
        for entry in values {
            match self.latest.get(&entry.signal_id) {
                Some(old) if old.same_bits(&entry.value) => {}
                _ => {
                    self.latest.insert(entry.signal_id, entry.value.clone());
                }
            }
        }
        self.latest_ts = ts;
        self.roll_window(ts);

        let mut bytes = 0u64;
        let mtu = self.mtu;
        let keyframe_interval = self.keyframe_interval_micros;
        let latest = &self.latest;
        for session in &mut self.sessions {
            bytes += send_due(session, latest, ts, keyframe_interval, mtu, udp);
        }
        self.window_bytes += bytes;
        self.sessions.retain(|s| !s.dead);
    }

    /// Once per second of robot time, compares bytes sent against the budget and lowers
    /// (or restores) the rate of the lowest-priority subscription.
    fn roll_window(&mut self, ts: u64) {
        if ts.saturating_sub(self.window_start) < 1_000_000 {
            return;
        }
        let used = self.window_bytes;
        self.last_window_bytes = used;
        let budget = self.budget_bytes_per_sec;
        if budget > 0 {
            if used > budget {
                // Lowest priority = newest session and newest subscription.
                'throttle: for session in self.sessions.iter_mut().rev() {
                    for sub in session.subs.iter_mut().rev() {
                        if matches!(sub.rate, Rate::Hz(_)) && sub.divisor < MAX_DIVISOR {
                            sub.divisor *= 2;
                            break 'throttle;
                        }
                    }
                }
            } else if used < budget / 2 {
                'restore: for session in self.sessions.iter_mut() {
                    for sub in session.subs.iter_mut() {
                        if sub.divisor > 1 {
                            sub.divisor /= 2;
                            break 'restore;
                        }
                    }
                }
            }
        }
        self.window_start = ts;
        self.window_bytes = 0;
    }

    /// Sends the latest values of signals newly matched by subscription `sub_id` (snapshot on
    /// subscribe). Only signals the subscriber has not yet received are sent.
    pub fn snapshot(&mut self, session_id: u32, sub_id: u32, udp: &UdpSocket) {
        let ts = self.latest_ts;
        let mtu = self.mtu;
        let latest = &self.latest;
        let Some(session) = self.sessions.iter_mut().find(|s| s.id == session_id) else { return };
        let Some(sub) = session.subs.iter().find(|s| s.id == sub_id) else { return };
        let mut entries = Vec::new();
        for id in &sub.matches {
            if session.sent.contains_key(id) {
                continue;
            }
            if let Some(v) = latest.get(id) {
                entries.push(Entry { signal_id: *id, value: v.clone() });
            }
        }
        if entries.is_empty() {
            return;
        }
        for e in &entries {
            session.sent.insert(e.signal_id, Sent { at: ts, value: e.value.clone() });
        }
        // The snapshot is the subscriber's baseline, so the periodic keyframe is not due yet.
        if session.last_keyframe.is_none() {
            session.last_keyframe = Some(ts);
        }
        let seq = next_seq(session);
        let bytes = send_frames(udp, session.udp, false, ts, seq, &entries, mtu);
        self.window_bytes += bytes;
    }

    pub fn send_marker(&self, marker: Marker, udp: &UdpSocket) {
        let mut w = Writer::new();
        Record::Marker(marker).encode(&mut w);
        for session in &self.sessions {
            if let Some(addr) = session.udp {
                let _ = udp.send_to(w.as_slice(), addr);
            }
        }
    }
}

fn next_seq(session: &mut Session) -> u32 {
    let seq = session.seq;
    session.seq = session.seq.wrapping_add(1);
    seq
}

fn match_signals(dict: &Dictionary, pattern_text: &str) -> Vec<u64> {
    let mut ids: Vec<u64> = dict
        .entries
        .iter()
        .filter(|e| pattern::matches(pattern_text, &e.path))
        .map(|e| e.signal_id)
        .collect();
    ids.sort_unstable();
    ids.dedup();
    ids
}

/// Decides which of the session's subscribed signals are due this cycle and sends one batched
/// frame (fragmented if needed). Returns bytes sent.
fn send_due(
    session: &mut Session,
    latest: &HashMap<u64, arcp_core::v2::Value>,
    ts: u64,
    keyframe_interval: u64,
    mtu: usize,
    udp: &UdpSocket,
) -> u64 {
    if session.udp.is_none() || session.subs.is_empty() {
        return 0;
    }
    let mut policy: BTreeMap<u64, Policy> = BTreeMap::new();
    for sub in &session.subs {
        let p = sub.effective();
        for id in &sub.matches {
            policy
                .entry(*id)
                .and_modify(|existing| *existing = existing.merge(p))
                .or_insert(p);
        }
    }
    let due_key = session.last_keyframe.is_none_or(|t| ts.saturating_sub(t) >= keyframe_interval);
    let mut entries = Vec::new();
    for (id, pol) in policy {
        let Some(value) = latest.get(&id) else { continue };
        let send = match session.sent.get(&id) {
            None => true,
            Some(prev) => {
                due_key
                    || match pol {
                        Policy::OnChange => !prev.value.same_bits(value),
                        Policy::Hz(hz) => (ts.saturating_sub(prev.at) as f64) >= 0.9 * 1_000_000.0 / hz,
                    }
            }
        };
        if send {
            session.sent.insert(id, Sent { at: ts, value: value.clone() });
            entries.push(Entry { signal_id: id, value: value.clone() });
        }
    }
    if entries.is_empty() {
        return 0;
    }
    if due_key {
        session.last_keyframe = Some(ts);
    }
    let seq = next_seq(session);
    send_frames(udp, session.udp, due_key, ts, seq, &entries, mtu)
}

/// Encodes entries into one or more frame records (each datagram-sized) and sends them.
pub(crate) fn send_frames(
    udp: &UdpSocket,
    to: Option<SocketAddr>,
    keyframe: bool,
    ts: u64,
    seq: u32,
    entries: &[Entry],
    mtu: usize,
) -> u64 {
    let Some(addr) = to else { return 0 };
    let encoded: Vec<Vec<u8>> = entries
        .iter()
        .map(|e| {
            let mut w = Writer::new();
            w.varint(e.signal_id);
            e.value.encode(&mut w);
            w.into_vec()
        })
        .collect();

    let budget = mtu.saturating_sub(FRAME_OVERHEAD).max(1);
    let mut chunks: Vec<Vec<usize>> = vec![Vec::new()];
    let mut used = 0usize;
    for (i, bytes) in encoded.iter().enumerate() {
        if used + bytes.len() > budget && !chunks.last().is_some_and(Vec::is_empty) {
            chunks.push(Vec::new());
            used = 0;
        }
        chunks.last_mut().expect("at least one chunk").push(i);
        used += bytes.len();
    }

    let count = chunks.len();
    let mut sent = 0u64;
    for (index, chunk) in chunks.iter().enumerate() {
        let fragmented = count > 1;
        let mut flags = 0u8;
        if keyframe {
            flags |= FRAME_FLAG_KEYFRAME;
        }
        if fragmented {
            flags |= FRAME_FLAG_FRAGMENT;
        }
        let mut payload = Writer::with_capacity(mtu);
        payload
            .u8(FRAME_VERSION)
            .u8(flags)
            .u64(ts)
            .u32(seq)
            .u16(if fragmented { index as u16 } else { 0 })
            .u16(if fragmented { count as u16 } else { 1 })
            .varint(chunk.len() as u64);
        for i in chunk {
            payload.bytes(&encoded[*i]);
        }
        let mut datagram = Writer::with_capacity(payload.len() + 12);
        datagram.record(RECORD_FRAME, payload.as_slice());
        if udp.send_to(datagram.as_slice(), addr).is_ok() {
            sent += datagram.len() as u64;
        }
    }
    sent
}

/// Writes one length-prefixed control message. Returns false when the socket is unusable.
pub(crate) fn send(writer: &Mutex<TcpStream>, request_id: u32, message: &ServerMessage) -> bool {
    let body = encode_server(request_id, message);
    let Ok(bytes) = frame(&body) else { return false };
    match writer.lock() {
        Ok(mut w) => w.write_all(&bytes).and_then(|()| w.flush()).is_ok(),
        Err(_) => false,
    }
}
