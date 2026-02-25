use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;

use arcp_core::{encode_update_into, SignalValue};

pub(crate) enum PublishMessage {
    Value { signal_id: u16, value: SignalValue },
}

pub(crate) fn run_realtime_loop(
    socket: UdpSocket,
    publish_rx: mpsc::Receiver<PublishMessage>,
    subscribers: Arc<Mutex<Vec<SocketAddr>>>,
    running: Arc<AtomicBool>,
) {
    let mut encoded = Vec::with_capacity(256);
    while running.load(Ordering::SeqCst) {
        let message = match publish_rx.recv_timeout(Duration::from_millis(100)) {
            Ok(message) => message,
            Err(mpsc::RecvTimeoutError::Timeout) => continue,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };

        let encode_ok = match message {
            PublishMessage::Value { signal_id, value } => {
                encode_update_into(signal_id, &value, &mut encoded).is_ok()
            }
        };
        if !encode_ok {
            continue;
        }

        let endpoints = match subscribers.lock() {
            Ok(list) => list.clone(),
            Err(_) => Vec::new(),
        };
        for endpoint in endpoints {
            let _ = socket.send_to(&encoded, endpoint);
        }
    }
}
