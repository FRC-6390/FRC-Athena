//! One TCP control connection: typed requests in, structured replies out (E4, E5, E6).

use std::io::Read;
use std::net::TcpStream;
use std::sync::Arc;

use arcp_core::v2::control::{
    decode_client, take_frame, ClientMessage, Rate, ReplyData, ServerMessage, Status,
    CONTROL_PROTOCOL_VERSION,
};
use arcp_core::v2::pattern;

use super::state::{send, Subscription};
use super::Inner;
use super::transport::ControlWriter;

pub(crate) fn serve(inner: Arc<Inner>, stream: TcpStream) {
    let _ = stream.set_nodelay(true);
    let Ok(peer) = stream.peer_addr() else { return };
    let Ok(writer) = ControlWriter::start(&stream) else { return };

    // Register the session and announce the schema before any other traffic.
    let session_id = {
        let mut state = inner.lock();
        let id = state.add_session(Arc::clone(&writer));
        let schema = state.schema_message();
        if !send(&writer, 0, &schema) {
            state.remove_session(id);
            return;
        }
        id
    };

    let mut conn = Connection { inner: Arc::clone(&inner), writer: Arc::clone(&writer), session_id, peer, hello_done: false };
    let mut reader = stream;
    let mut buf: Vec<u8> = Vec::new();
    let mut chunk = [0u8; 4096];
    'read: loop {
        match reader.read(&mut chunk) {
            Ok(0) | Err(_) => break,
            Ok(n) => buf.extend_from_slice(&chunk[..n]),
        }
        loop {
            let (consumed, body) = match take_frame(&buf) {
                Ok(Some((consumed, body))) => (consumed, body.to_vec()),
                Ok(None) => break,
                Err(_) => break 'read,
            };
            buf.drain(..consumed);
            match decode_client(&body) {
                Ok((request_id, message)) => {
                    if !conn.handle(request_id, message) {
                        break 'read;
                    }
                }
                Err(_) => {
                    send(&writer, 0, &reply(Status::Error, "malformed request", ReplyData::None));
                    break 'read;
                }
            }
        }
    }

    inner.lock().remove_session(session_id);
    writer.finish();
}

struct Connection {
    inner: Arc<Inner>,
    writer: Arc<ControlWriter>,
    session_id: u32,
    peer: std::net::SocketAddr,
    hello_done: bool,
}

impl Connection {
    /// Handles one request. Returns false when the connection should close.
    fn handle(&mut self, request_id: u32, message: ClientMessage) -> bool {
        if let ClientMessage::TimePing { client_micros } = message {
            let server_micros = self.inner.elapsed_micros();
            return send(&self.writer, request_id, &ServerMessage::TimePong { client_micros, server_micros });
        }
        if let ClientMessage::Hello { client_name, protocol_version, udp_port } = message {
            if protocol_version != CONTROL_PROTOCOL_VERSION {
                return self.reply(request_id, Status::Error, "unsupported protocol version", ReplyData::None);
            }
            {
                let mut state = self.inner.lock();
                if let Some(session) = state.session_mut(self.session_id) {
                    let mut addr = self.peer;
                    addr.set_port(udp_port);
                    session.udp = Some(addr);
                    session.name = client_name;
                }
            }
            self.hello_done = true;
            return self.reply(request_id, Status::Ok, "", ReplyData::Session { session_id: self.session_id });
        }
        // Schema is public on connect, so a schema request does not need a session yet.
        if let ClientMessage::SchemaRequest = message {
            let schema = self.inner.lock().schema_message();
            return send(&self.writer, request_id, &schema);
        }
        if !self.hello_done {
            return self.reply(request_id, Status::Error, "hello required", ReplyData::None);
        }
        match message {
            ClientMessage::Subscribe { pattern: glob, rate } => self.subscribe(request_id, glob, rate),
            ClientMessage::Unsubscribe { subscription_id } => {
                let mut state = self.inner.lock();
                let removed = state.session_mut(self.session_id).is_some_and(|s| {
                    let before = s.subs.len();
                    s.subs.retain(|sub| sub.id != subscription_id);
                    s.subs.len() != before
                });
                drop(state);
                if removed {
                    self.reply(request_id, Status::Ok, "", ReplyData::None)
                } else {
                    self.reply(request_id, Status::Error, "no such subscription", ReplyData::None)
                }
            }
            ClientMessage::Write { signal_id, value } => {
                // The robot validates; the server never pre-blocks writes.
                let reply = self.inner.handler.write(signal_id, value);
                send(&self.writer, request_id, &ServerMessage::WriteReply(reply))
            }
            ClientMessage::ActionRun { signal_id } => {
                let result = self.inner.handler.action(signal_id, true);
                self.robot_reply(request_id, result)
            }
            ClientMessage::ActionCancel { signal_id } => {
                let result = self.inner.handler.action(signal_id, false);
                self.robot_reply(request_id, result)
            }
            ClientMessage::SchemaRequest => true,
            ClientMessage::FileList => match self.files() {
                Ok(roots) => match roots.list() {
                    Ok(list) => self.reply(request_id, Status::Ok, "", ReplyData::Files(list)),
                    Err(e) => self.reply(request_id, Status::Error, e, ReplyData::None),
                },
                Err(e) => self.reply(request_id, Status::Error, e, ReplyData::None),
            },
            ClientMessage::FileDownload { name, offset, max_len } => match self.files() {
                Ok(roots) => match roots.download(&name, offset, max_len) {
                    Ok((eof, data)) => self.reply(request_id, Status::Ok, "", ReplyData::Chunk { eof, data }),
                    Err(e) => self.reply(request_id, Status::Error, e, ReplyData::None),
                },
                Err(e) => self.reply(request_id, Status::Error, e, ReplyData::None),
            },
            ClientMessage::FileDelete { name } => match self.files() {
                Ok(roots) => match roots.delete(&name) {
                    Ok(()) => self.reply(request_id, Status::Ok, "", ReplyData::None),
                    Err(e) => self.reply(request_id, Status::Error, e, ReplyData::None),
                },
                Err(e) => self.reply(request_id, Status::Error, e, ReplyData::None),
            },
            ClientMessage::ReplaySubscribe => {
                if let Some(session) = self.inner.lock().session_mut(self.session_id) {
                    session.replay = true;
                }
                self.reply(request_id, Status::Ok, "", ReplyData::None)
            }
            ClientMessage::Backfill { from, to } => match self.inner.handler.backfill(from, to) {
                Ok(data) => self.reply(request_id, Status::Ok, "", ReplyData::Records(data)),
                Err(reason) => self.reply(request_id, Status::Error, &reason, ReplyData::None),
            },
            ClientMessage::Hello { .. } | ClientMessage::TimePing { .. } => true,
        }
    }

    fn subscribe(&mut self, request_id: u32, glob: String, rate: Rate) -> bool {
        if let Err(e) = pattern::validate(&glob).and_then(|()| rate.validate()) {
            return self.reply(request_id, Status::Error, e, ReplyData::None);
        }
        let mut state = self.inner.lock();
        let Some(session) = state.session_mut(self.session_id) else { return false };
        let sub_id = session.next_sub_id;
        session.next_sub_id += 1;
        let matches = state
            .dictionary
            .entries
            .iter()
            .filter(|e| pattern::matches(&glob, &e.path))
            .map(|e| e.signal_id)
            .collect();
        let Some(session) = state.session_mut(self.session_id) else { return false };
        session.subs.push(Subscription { id: sub_id, pattern: glob, rate, divisor: 1, matches });
        // Latest values for newly matched signals, once.
        state.snapshot(self.session_id, sub_id, &self.inner.udp);
        drop(state);
        self.reply(request_id, Status::Ok, "", ReplyData::Subscription { subscription_id: sub_id })
    }

    fn robot_reply(&mut self, request_id: u32, result: Result<(), String>) -> bool {
        match result {
            Ok(()) => self.reply(request_id, Status::Ok, "", ReplyData::None),
            Err(reason) => self.reply(request_id, Status::Rejected, &reason, ReplyData::None),
        }
    }

    fn files(&self) -> Result<&super::files::Roots, &'static str> {
        if self.inner.roots.enabled() {
            Ok(&self.inner.roots)
        } else {
            Err("file service not configured")
        }
    }

    fn reply(&self, request_id: u32, status: Status, reason: &str, data: ReplyData) -> bool {
        send(&self.writer, request_id, &reply(status, reason, data))
    }
}

fn reply(status: Status, reason: &str, data: ReplyData) -> ServerMessage {
    ServerMessage::Reply { status, reason: reason.to_owned(), data }
}

