//! Request queue between the server's connection threads and the robot thread.
//!
//! A client `Write` or `Action` blocks its connection thread on a one-shot channel. The robot
//! thread drains the queue once per cycle (`poll`) and answers (`apply_replies`). If no answer
//! arrives within [`REPLY_TIMEOUT`] the client gets a rejection, so a stopped robot loop never
//! wedges a connection.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use arcp_core::v2::control::WriteReply;
use arcp_core::v2::record::Dictionary;
use arcp_core::v2::{Entry, Value};
use arcp_server::v2::RobotHandler;

use crate::marshal::{self, put_u64, put_u8, Reader};

pub const OP_WRITE: u8 = 1;
pub const OP_ACTION_RUN: u8 = 2;
pub const OP_ACTION_CANCEL: u8 = 3;
/// Backfill: `signal_id` carries the first wanted frame sequence, the value the last one (I64).
pub const OP_BACKFILL: u8 = 4;

/// Requests waiting for the robot thread. Beyond this, new requests are rejected at once.
pub const MAX_PENDING: usize = 256;
/// How long a client waits for the robot thread before its request is rejected.
pub const REPLY_TIMEOUT: Duration = Duration::from_millis(250);

enum Answer {
    Write(WriteReply),
    Action(Result<(), String>),
    Backfill(Result<Vec<u8>, String>),
}

struct Waiter {
    signal_id: u64,
    tx: SyncSender<Answer>,
}

struct Queue {
    pending: VecDeque<Request>,
    waiting: HashMap<u64, Waiter>,
    next_id: u64,
    writable: HashSet<u64>,
    /// Last value the robot published (or accepted) for each writable signal. Used as the
    /// `actualValue` of a rejection when the robot did not answer in time.
    last: HashMap<u64, Value>,
}

struct Request {
    id: u64,
    op: u8,
    signal_id: u64,
    value: Option<Value>,
}

pub struct Bridge {
    queue: Mutex<Queue>,
}

impl Bridge {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            queue: Mutex::new(Queue {
                pending: VecDeque::new(),
                waiting: HashMap::new(),
                next_id: 1,
                writable: HashSet::new(),
                last: HashMap::new(),
            }),
        })
    }

    fn lock(&self) -> MutexGuard<'_, Queue> {
        self.queue.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Records the writable signals (dictionary flag bit0) so their values are remembered.
    pub fn set_dictionary(&self, dictionary: &Dictionary) {
        let mut q = self.lock();
        q.writable = dictionary
            .entries
            .iter()
            .filter(|e| e.flags & 0x01 != 0)
            .map(|e| e.signal_id)
            .collect();
        let writable = q.writable.clone();
        q.last.retain(|id, _| writable.contains(id));
    }

    pub fn note_cycle(&self, values: &[Entry]) {
        let mut q = self.lock();
        if q.writable.is_empty() {
            return;
        }
        for e in values {
            if q.writable.contains(&e.signal_id) {
                q.last.insert(e.signal_id, e.value.clone());
            }
        }
    }

    /// Queues a request and waits for the robot's answer. `None` when no answer arrived in time.
    fn submit(&self, op: u8, signal_id: u64, value: Option<Value>) -> Option<Answer> {
        let (tx, rx) = sync_channel(1);
        let id = {
            let mut q = self.lock();
            q.next_id += 1;
            let id = q.next_id;
            q.waiting.insert(id, Waiter { signal_id, tx });
            q.pending.push_back(Request {
                id,
                op,
                signal_id,
                value,
            });
            id
        };
        match rx.recv_timeout(REPLY_TIMEOUT) {
            Ok(answer) => Some(answer),
            Err(_) => {
                self.lock().waiting.remove(&id);
                None
            }
        }
    }

    fn remember(&self, signal_id: u64) -> Option<Value> {
        self.lock().last.get(&signal_id).cloned()
    }

    fn full(&self) -> bool {
        self.lock().pending.len() >= MAX_PENDING
    }

    /// Writes the pending requests into `buf` (see `marshal`). Returns the bytes written, or the
    /// negative size needed for the first request when `buf` is too small. Written requests are
    /// removed from the queue only when they fit.
    pub fn poll(&self, buf: &mut [u8]) -> i32 {
        if buf.len() < 4 {
            return -4;
        }
        let mut q = self.lock();
        let mut body = Vec::new();
        let mut count: u32 = 0;
        for req in q.pending.iter() {
            let mut rec = Vec::new();
            put_u64(&mut rec, req.id);
            put_u8(&mut rec, req.op);
            put_u64(&mut rec, req.signal_id);
            if let Some(v) = &req.value {
                marshal::put_value(&mut rec, v);
            }
            if 4 + body.len() + rec.len() > buf.len() {
                if count == 0 {
                    return -((4 + rec.len()) as i32);
                }
                break;
            }
            body.extend_from_slice(&rec);
            count += 1;
        }
        buf[..4].copy_from_slice(&count.to_le_bytes());
        buf[4..4 + body.len()].copy_from_slice(&body);
        q.pending.drain(..count as usize);
        (4 + body.len()) as i32
    }

    /// Applies answers from the robot thread. Unknown or late request ids are ignored.
    pub fn apply_replies(&self, data: &[u8]) -> Result<(), String> {
        let mut r = Reader::new(data);
        let count = r.u32()?;
        let mut q = self.lock();
        for _ in 0..count {
            let id = r.u64()?;
            let op = r.u8()?;
            let accepted = r.u8()? != 0;
            let reason = r.string()?;
            let value = if op == OP_WRITE {
                Some(r.value()?)
            } else {
                None
            };
            // Backfill replies carry the record groups as a length-prefixed byte block.
            let backfill = if op == OP_BACKFILL {
                let n = r.u32()? as usize;
                Some(r.bytes(n)?.to_vec())
            } else {
                None
            };
            let Some(waiter) = q.waiting.remove(&id) else {
                continue;
            };
            if let Some(data) = backfill {
                let answer = if accepted { Ok(data) } else { Err(reason) };
                let _ = waiter.tx.try_send(Answer::Backfill(answer));
                continue;
            }
            let answer = match value {
                Some(actual_value) => {
                    if accepted && q.writable.contains(&waiter.signal_id) {
                        q.last.insert(waiter.signal_id, actual_value.clone());
                    }
                    Answer::Write(WriteReply {
                        accepted,
                        reason,
                        actual_value,
                    })
                }
                None => Answer::Action(if accepted { Ok(()) } else { Err(reason) }),
            };
            let _ = waiter.tx.try_send(answer);
        }
        if !r.finished() {
            return Err("trailing bytes in reply buffer".into());
        }
        Ok(())
    }
}

/// Adapter the server calls from connection threads.
pub struct BridgeHandler(pub Arc<Bridge>);

impl RobotHandler for BridgeHandler {
    fn write(&self, signal_id: u64, value: Value) -> WriteReply {
        let fallback = self.0.remember(signal_id).unwrap_or_else(|| value.clone());
        if self.0.full() {
            return WriteReply::rejected("robot busy: request queue full", fallback);
        }
        match self.0.submit(OP_WRITE, signal_id, Some(value)) {
            Some(Answer::Write(reply)) => reply,
            Some(_) => WriteReply::rejected("internal: wrong answer kind", fallback),
            None => WriteReply::rejected("robot did not answer (no robot cycle)", fallback),
        }
    }

    fn backfill(&self, from: u64, to: u64) -> Result<Vec<u8>, String> {
        if self.0.full() {
            return Err("robot busy: request queue full".into());
        }
        match self.0.submit(OP_BACKFILL, from, Some(Value::I64(to as i64))) {
            Some(Answer::Backfill(result)) => result,
            Some(_) => Err("internal: wrong answer kind".into()),
            None => Err("robot did not answer (no robot cycle)".into()),
        }
    }

    fn action(&self, signal_id: u64, run: bool) -> Result<(), String> {
        if self.0.full() {
            return Err("robot busy: request queue full".into());
        }
        let op = if run { OP_ACTION_RUN } else { OP_ACTION_CANCEL };
        match self.0.submit(op, signal_id, None) {
            Some(Answer::Action(result)) => result,
            Some(_) => Err("internal: wrong answer kind".into()),
            None => Err("robot did not answer (no robot cycle)".into()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::marshal::{put_u32, put_u64, put_u8, put_value, Reader};
    use std::thread;

    fn dictionary(writable: bool) -> Dictionary {
        Dictionary {
            entries: vec![arcp_core::v2::record::DictEntry {
                signal_id: 1,
                path: "robot/turret/angle".into(),
                type_tag: 3,
                unit: String::new(),
                role: 1,
                flags: if writable { 0x01 } else { 0 },
                lock_group: String::new(),
                struct_schema_id: 0,
            }],
        }
    }

    /// Reply record for a write (op 1) or action (op 2/3), as the Java side encodes it.
    fn reply_bytes(
        id: u64,
        op: u8,
        accepted: bool,
        reason: &str,
        value: Option<&Value>,
    ) -> Vec<u8> {
        let mut out = Vec::new();
        put_u32(&mut out, 1);
        put_u64(&mut out, id);
        put_u8(&mut out, op);
        put_u8(&mut out, u8::from(accepted));
        put_u32(&mut out, reason.len() as u32);
        out.extend_from_slice(reason.as_bytes());
        if let Some(v) = value {
            put_value(&mut out, v);
        }
        out
    }

    #[test]
    fn poll_hands_out_requests_and_reply_completes_them() {
        let bridge = Bridge::new();
        bridge.set_dictionary(&dictionary(true));
        bridge.note_cycle(&[Entry {
            signal_id: 1,
            value: Value::F64(1.5),
        }]);

        let worker = {
            let handler = BridgeHandler(Arc::clone(&bridge));
            thread::spawn(move || handler.write(1, Value::F64(2.0)))
        };

        // Robot side: poll until the request appears, then answer it.
        let mut buf = vec![0u8; 1024];
        let mut written = 0;
        for _ in 0..200 {
            written = bridge.poll(&mut buf);
            if written > 4 {
                break;
            }
            thread::sleep(Duration::from_millis(1));
        }
        assert!(written > 4, "request never queued");
        let mut r = Reader::new(&buf[..written as usize]);
        assert_eq!(r.u32().unwrap(), 1);
        let id = r.u64().unwrap();
        assert_eq!(r.u8().unwrap(), OP_WRITE);
        assert_eq!(r.u64().unwrap(), 1);
        assert_eq!(r.value().unwrap(), Value::F64(2.0));

        bridge
            .apply_replies(&reply_bytes(
                id,
                OP_WRITE,
                false,
                "rejected: locked",
                Some(&Value::F64(1.5)),
            ))
            .unwrap();
        let reply = worker.join().unwrap();
        assert!(!reply.accepted);
        assert_eq!(reply.reason, "rejected: locked");
        assert_eq!(reply.actual_value, Value::F64(1.5));
        assert_eq!(bridge.poll(&mut buf), 4, "queue should be empty");
    }

    #[test]
    fn unanswered_write_times_out_with_last_known_value() {
        let bridge = Bridge::new();
        bridge.set_dictionary(&dictionary(true));
        bridge.note_cycle(&[Entry {
            signal_id: 1,
            value: Value::F64(3.0),
        }]);
        let handler = BridgeHandler(Arc::clone(&bridge));
        let reply = handler.write(1, Value::F64(9.0));
        assert!(!reply.accepted);
        assert!(reply.reason.contains("did not answer"));
        assert_eq!(reply.actual_value, Value::F64(3.0));
    }

    #[test]
    fn small_buffer_reports_needed_size_without_dequeuing() {
        let bridge = Bridge::new();
        bridge.set_dictionary(&dictionary(true));
        let handler = BridgeHandler(Arc::clone(&bridge));
        let worker = thread::spawn(move || handler.write(1, Value::Str("x".repeat(100))));
        let mut big = vec![0u8; 4096];
        let mut small = vec![0u8; 8];
        let mut needed = 0;
        for _ in 0..200 {
            needed = bridge.poll(&mut small);
            if needed < 0 {
                break;
            }
            thread::sleep(Duration::from_millis(1));
        }
        assert!(needed < -4, "expected a negative needed size, got {needed}");
        let written = bridge.poll(&mut big);
        assert!(written > 100);
        let id = u64::from_le_bytes(big[4..12].try_into().unwrap());
        bridge
            .apply_replies(&reply_bytes(
                id,
                OP_WRITE,
                true,
                "",
                Some(&Value::Str("x".into())),
            ))
            .unwrap();
        let reply = worker.join().unwrap();
        assert!(reply.accepted);
    }
}
