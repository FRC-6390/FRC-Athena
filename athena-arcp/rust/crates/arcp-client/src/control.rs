//! TCP control channel: one reader thread that routes replies to waiting requests and applies
//! schema pushes to the model.

use std::io::Read;
use std::net::TcpStream;
use std::sync::Arc;

use arcp_core::v2::control::{decode_server, take_frame, ServerMessage};

use crate::client::{lock, Event, Shared};

pub(crate) fn run_reader(shared: Arc<Shared>, mut stream: TcpStream) {
    let mut buf: Vec<u8> = Vec::new();
    let mut chunk = vec![0u8; 8192];
    let reason: String = 'read: loop {
        match stream.read(&mut chunk) {
            Ok(0) => break 'read "connection closed by robot".to_owned(),
            Ok(n) => buf.extend_from_slice(&chunk[..n]),
            Err(e) => break 'read format!("control read failed: {e}"),
        }
        loop {
            let (consumed, decoded) = match take_frame(&buf) {
                Ok(Some((consumed, body))) => (consumed, decode_server(body)),
                Ok(None) => break,
                Err(e) => break 'read format!("protocol error: {e}"),
            };
            buf.drain(..consumed);
            match decoded {
                Ok((id, message)) => dispatch(&shared, id, message),
                Err(e) => break 'read format!("protocol error: {e}"),
            }
        }
    };
    shared.connected.store(false, std::sync::atomic::Ordering::SeqCst);
    // Dropping the senders wakes every waiting request with a disconnect.
    lock(&shared.pending).clear();
    let _ = shared.events.send(Event::Disconnected { reason });
}

fn dispatch(shared: &Shared, id: u32, message: ServerMessage) {
    if let ServerMessage::Schema { dictionary, schemas } = &message {
        lock(&shared.model).set_dictionary(dictionary.clone(), schemas.clone());
        let _ = shared.events.send(Event::Dictionary { dictionary: dictionary.clone(), schemas: schemas.clone() });
    }
    // Request id 0 is a server push (schema change).
    if id != 0 {
        if let Some(tx) = lock(&shared.pending).remove(&id) {
            let _ = tx.send(message);
        }
    }
}
