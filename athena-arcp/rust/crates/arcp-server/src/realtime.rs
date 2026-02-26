use std::collections::{HashMap, HashSet};
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
    let mut latest_frames: HashMap<u16, Vec<u8>> = HashMap::new();
    let mut known_subscribers: HashSet<SocketAddr> = HashSet::new();
    while running.load(Ordering::SeqCst) {
        let message = match publish_rx.recv_timeout(Duration::from_millis(100)) {
            Ok(message) => Some(message),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };

        let endpoints = match subscribers.lock() {
            Ok(list) => list.clone(),
            Err(_) => Vec::new(),
        };
        let endpoint_set: HashSet<SocketAddr> = endpoints.iter().copied().collect();

        // New subscribers should immediately receive the latest known value for every signal so
        // dashboards that connect after startup do not stay at null until the next value change.
        for endpoint in endpoint_set.difference(&known_subscribers) {
            for frame in latest_frames.values() {
                let _ = socket.send_to(frame, endpoint);
            }
        }
        known_subscribers = endpoint_set;

        let Some(message) = message else {
            continue;
        };

        let encode_ok = match message {
            PublishMessage::Value { signal_id, value } => {
                if !encode_update_into(signal_id, &value, &mut encoded).is_ok() {
                    false
                } else {
                    latest_frames.insert(signal_id, encoded.clone());
                    true
                }
            }
        };
        if !encode_ok {
            continue;
        }

        for endpoint in endpoints {
            let _ = socket.send_to(&encoded, endpoint);
        }
    }
}
