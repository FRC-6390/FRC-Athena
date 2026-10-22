//! A bounded, ordered TCP outbox. No caller writes to a socket while holding server state.

use std::io::{self, Write};
use std::net::{Shutdown, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use arcp_core::v2::control::{encode_server, frame, ServerMessage};

const MAX_MESSAGES: usize = 32;
const MAX_QUEUED_BYTES: usize = 2 * 1024 * 1024;
const WRITE_TIMEOUT: Duration = Duration::from_millis(500);

pub(crate) struct ControlWriter {
    queue: SyncSender<Option<Vec<u8>>>,
    queued_bytes: Arc<AtomicUsize>,
    failed: Arc<AtomicBool>,
    shutdown: TcpStream,
}

impl ControlWriter {
    pub fn start(stream: &TcpStream) -> io::Result<Arc<Self>> {
        let mut socket = stream.try_clone()?;
        socket.set_write_timeout(Some(WRITE_TIMEOUT))?;
        socket.set_nodelay(true)?;
        let shutdown = stream.try_clone()?;
        let (queue, incoming) = sync_channel::<Option<Vec<u8>>>(MAX_MESSAGES);
        let queued_bytes = Arc::new(AtomicUsize::new(0));
        let failed = Arc::new(AtomicBool::new(false));
        let bytes = Arc::clone(&queued_bytes);
        let stopped = Arc::clone(&failed);
        thread::Builder::new().name("arcp-v2-writer".into()).spawn(move || {
            while let Ok(Some(message)) = incoming.recv() {
                if stopped.load(Ordering::Acquire) {
                    break;
                }
                let result = socket.write_all(&message).and_then(|()| socket.flush());
                bytes.fetch_sub(message.len(), Ordering::AcqRel);
                if result.is_err() {
                    break;
                }
            }
            stopped.store(true, Ordering::Release);
            // Also wakes a connection thread waiting in read() when its outbox fails.
            let _ = socket.shutdown(Shutdown::Both);
        })?;
        Ok(Arc::new(Self { queue, queued_bytes, failed, shutdown }))
    }

    pub fn send(&self, request_id: u32, message: &ServerMessage) -> bool {
        if self.failed.load(Ordering::Acquire) {
            return false;
        }
        let body = encode_server(request_id, message);
        let Ok(bytes) = frame(&body) else {
            self.close();
            return false;
        };
        self.enqueue(bytes)
    }

    fn enqueue(&self, bytes: Vec<u8>) -> bool {
        let len = bytes.len();
        let mut used = self.queued_bytes.load(Ordering::Acquire);
        loop {
            let Some(next) = used.checked_add(len).filter(|&next| next <= MAX_QUEUED_BYTES) else {
                self.close();
                return false;
            };
            match self.queued_bytes.compare_exchange_weak(used, next, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => break,
                Err(current) => used = current,
            }
        }
        if self.queue.try_send(Some(bytes)).is_err() {
            self.queued_bytes.fetch_sub(len, Ordering::AcqRel);
            self.close();
            return false;
        }
        true
    }

    /// Close after already accepted replies (for example a malformed-request error) are sent.
    pub fn finish(&self) {
        if self.failed.load(Ordering::Acquire) || self.queue.try_send(None).is_err() {
            self.close();
        }
    }

    pub fn close(&self) {
        self.failed.store(true, Ordering::Release);
        let _ = self.shutdown.shutdown(Shutdown::Both);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;
    use std::time::Instant;

    #[test]
    fn full_outbox_disconnects_without_waiting_for_the_writer() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let _reader = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let (socket, _) = listener.accept().unwrap();
        // No consumer: a deterministic full queue, independent of OS socket buffering and timing.
        let (queue, _incoming) = sync_channel(MAX_MESSAGES);
        let writer = ControlWriter {
            queue,
            queued_bytes: Arc::new(AtomicUsize::new(0)),
            failed: Arc::new(AtomicBool::new(false)),
            shutdown: socket,
        };
        for _ in 0..MAX_MESSAGES {
            assert!(writer.enqueue(vec![1]));
        }
        let start = Instant::now();
        assert!(!writer.enqueue(vec![2]));
        assert!(start.elapsed() < Duration::from_millis(100));
        assert!(writer.failed.load(Ordering::Acquire));
    }

    #[test]
    fn byte_limit_covers_large_messages_without_allocating_more_queue_slots() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let _reader = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let (socket, _) = listener.accept().unwrap();
        let (queue, _incoming) = sync_channel(MAX_MESSAGES);
        let writer = ControlWriter {
            queue,
            queued_bytes: Arc::new(AtomicUsize::new(MAX_QUEUED_BYTES)),
            failed: Arc::new(AtomicBool::new(false)),
            shutdown: socket,
        };
        assert!(!writer.enqueue(vec![0]));
        assert_eq!(writer.queued_bytes.load(Ordering::Acquire), MAX_QUEUED_BYTES);
    }
}
