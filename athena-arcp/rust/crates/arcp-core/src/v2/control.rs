//! Reliable control channel messages (E4): typed binary messages over TCP.
//!
//! Stream framing: `u32 LE bodyLength`, then the body. Body: `u8 kind`, `u32 requestId`
//! (little-endian), then the kind-specific payload. Replies echo the request id. Server pushes
//! (schema changes) use request id 0.

use super::record::{read_dictionary, read_struct_schema, write_dictionary, write_struct_schema, Dictionary, StructSchema};
use super::value::Value;
use super::wire::{Reader, WireError, WireResult, Writer};

/// Control protocol version announced in `Hello`.
pub const CONTROL_PROTOCOL_VERSION: u8 = 2;

/// Largest accepted message body (protects against corrupt length prefixes).
pub const MAX_CONTROL_BODY: usize = 1 << 20;

/// Largest file chunk returned by a download request.
pub const MAX_FILE_CHUNK: u32 = 60_000;

pub mod kind {
    pub const HELLO: u8 = 0x01;
    pub const SUBSCRIBE: u8 = 0x10;
    pub const UNSUBSCRIBE: u8 = 0x11;
    pub const WRITE: u8 = 0x20;
    pub const ACTION_RUN: u8 = 0x21;
    pub const ACTION_CANCEL: u8 = 0x22;
    pub const TIME_PING: u8 = 0x30;
    pub const SCHEMA_REQUEST: u8 = 0x40;
    pub const FILE_LIST: u8 = 0x50;
    pub const FILE_DOWNLOAD: u8 = 0x51;
    pub const FILE_DELETE: u8 = 0x52;

    pub const REPLY: u8 = 0x80;
    pub const SCHEMA: u8 = 0x81;
    pub const TIME_PONG: u8 = 0x82;
    pub const WRITE_REPLY: u8 = 0x83;
}

/// Publish rate of a subscription.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Rate {
    /// Send when the value differs from what the subscriber last received.
    OnChange,
    /// Send at most this many times per second (value sent even if unchanged).
    Hz(f64),
}

impl Rate {
    pub fn validate(&self) -> Result<(), &'static str> {
        match self {
            Rate::OnChange => Ok(()),
            Rate::Hz(hz) if hz.is_finite() && *hz > 0.0 && *hz <= 1000.0 => Ok(()),
            Rate::Hz(_) => Err("rate must be a finite value in (0, 1000] Hz"),
        }
    }

    fn encode(&self, w: &mut Writer) {
        match self {
            Rate::OnChange => {
                w.u8(0);
            }
            Rate::Hz(hz) => {
                w.u8(1).f64(*hz);
            }
        }
    }

    fn decode(r: &mut Reader<'_>) -> WireResult<Rate> {
        match r.u8()? {
            0 => Ok(Rate::OnChange),
            1 => Ok(Rate::Hz(r.f64()?)),
            _ => Err(WireError::Invalid("rate tag")),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ClientMessage {
    /// Opens the session. `udp_port` is the client's UDP port for realtime frames; the server uses
    /// the TCP peer address as the host.
    Hello { client_name: String, protocol_version: u8, udp_port: u16 },
    Subscribe { pattern: String, rate: Rate },
    Unsubscribe { subscription_id: u32 },
    /// Forwarded to the robot unchanged. The robot validates and replies.
    Write { signal_id: u64, value: Value },
    ActionRun { signal_id: u64 },
    ActionCancel { signal_id: u64 },
    TimePing { client_micros: u64 },
    SchemaRequest,
    FileList,
    FileDownload { name: String, offset: u64, max_len: u32 },
    FileDelete { name: String },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    Ok = 0,
    /// The robot refused the request. `reason` says why (for example a lock state).
    Rejected = 1,
    /// The request was malformed or could not be performed.
    Error = 2,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FileInfo {
    pub name: String,
    pub size: u64,
    pub modified_unix_secs: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ReplyData {
    None,
    Subscription { subscription_id: u32 },
    Session { session_id: u32 },
    Files(Vec<FileInfo>),
    Chunk { eof: bool, data: Vec<u8> },
}

/// Reply to a `Write` request: `{accepted, reason, actualValue}`. `actual_value` is the value the
/// robot holds after deciding, so a client can restore the displayed value on rejection.
#[derive(Debug, Clone, PartialEq)]
pub struct WriteReply {
    pub accepted: bool,
    /// Empty when accepted.
    pub reason: String,
    pub actual_value: Value,
}

impl WriteReply {
    pub fn accepted(actual_value: Value) -> Self {
        Self { accepted: true, reason: String::new(), actual_value }
    }

    pub fn rejected(reason: impl Into<String>, actual_value: Value) -> Self {
        Self { accepted: false, reason: reason.into(), actual_value }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ServerMessage {
    Reply { status: Status, reason: String, data: ReplyData },
    WriteReply(WriteReply),
    /// Full dictionary and struct schemas. Sent after `SchemaRequest` and pushed on change.
    Schema { dictionary: Dictionary, schemas: Vec<StructSchema> },
    TimePong { client_micros: u64, server_micros: u64 },
}

/// Encodes a client body (no length prefix).
pub fn encode_client(request_id: u32, message: &ClientMessage) -> Vec<u8> {
    let mut w = Writer::new();
    match message {
        ClientMessage::Hello { client_name, protocol_version, udp_port } => {
            header(&mut w, kind::HELLO, request_id);
            w.string(client_name).u8(*protocol_version).u16(*udp_port);
        }
        ClientMessage::Subscribe { pattern, rate } => {
            header(&mut w, kind::SUBSCRIBE, request_id);
            w.string(pattern);
            rate.encode(&mut w);
        }
        ClientMessage::Unsubscribe { subscription_id } => {
            header(&mut w, kind::UNSUBSCRIBE, request_id);
            w.u32(*subscription_id);
        }
        ClientMessage::Write { signal_id, value } => {
            header(&mut w, kind::WRITE, request_id);
            w.varint(*signal_id);
            value.encode(&mut w);
        }
        ClientMessage::ActionRun { signal_id } => {
            header(&mut w, kind::ACTION_RUN, request_id);
            w.varint(*signal_id);
        }
        ClientMessage::ActionCancel { signal_id } => {
            header(&mut w, kind::ACTION_CANCEL, request_id);
            w.varint(*signal_id);
        }
        ClientMessage::TimePing { client_micros } => {
            header(&mut w, kind::TIME_PING, request_id);
            w.u64(*client_micros);
        }
        ClientMessage::SchemaRequest => header(&mut w, kind::SCHEMA_REQUEST, request_id),
        ClientMessage::FileList => header(&mut w, kind::FILE_LIST, request_id),
        ClientMessage::FileDownload { name, offset, max_len } => {
            header(&mut w, kind::FILE_DOWNLOAD, request_id);
            w.string(name).u64(*offset).u32(*max_len);
        }
        ClientMessage::FileDelete { name } => {
            header(&mut w, kind::FILE_DELETE, request_id);
            w.string(name);
        }
    }
    w.into_vec()
}

/// Decodes a client body. Returns `(requestId, message)`.
pub fn decode_client(body: &[u8]) -> WireResult<(u32, ClientMessage)> {
    let mut r = Reader::new(body);
    let k = r.u8()?;
    let request_id = r.u32()?;
    let message = match k {
        kind::HELLO => ClientMessage::Hello {
            client_name: r.string()?,
            protocol_version: r.u8()?,
            udp_port: r.u16()?,
        },
        kind::SUBSCRIBE => {
            let pattern = r.string()?;
            let rate = Rate::decode(&mut r)?;
            ClientMessage::Subscribe { pattern, rate }
        }
        kind::UNSUBSCRIBE => ClientMessage::Unsubscribe { subscription_id: r.u32()? },
        kind::WRITE => ClientMessage::Write { signal_id: r.varint()?, value: Value::decode(&mut r)? },
        kind::ACTION_RUN => ClientMessage::ActionRun { signal_id: r.varint()? },
        kind::ACTION_CANCEL => ClientMessage::ActionCancel { signal_id: r.varint()? },
        kind::TIME_PING => ClientMessage::TimePing { client_micros: r.u64()? },
        kind::SCHEMA_REQUEST => ClientMessage::SchemaRequest,
        kind::FILE_LIST => ClientMessage::FileList,
        kind::FILE_DOWNLOAD => ClientMessage::FileDownload {
            name: r.string()?,
            offset: r.u64()?,
            max_len: r.u32()?,
        },
        kind::FILE_DELETE => ClientMessage::FileDelete { name: r.string()? },
        _ => return Err(WireError::Invalid("unknown client message kind")),
    };
    expect_end(&r)?;
    Ok((request_id, message))
}

/// Encodes a server body (no length prefix).
pub fn encode_server(request_id: u32, message: &ServerMessage) -> Vec<u8> {
    let mut w = Writer::new();
    match message {
        ServerMessage::Reply { status, reason, data } => {
            header(&mut w, kind::REPLY, request_id);
            w.u8(*status as u8).string(reason);
            encode_reply_data(&mut w, data);
        }
        ServerMessage::Schema { dictionary, schemas } => {
            header(&mut w, kind::SCHEMA, request_id);
            write_dictionary(&mut w, dictionary);
            w.varint(schemas.len() as u64);
            for s in schemas {
                write_struct_schema(&mut w, s);
            }
        }
        ServerMessage::TimePong { client_micros, server_micros } => {
            header(&mut w, kind::TIME_PONG, request_id);
            w.u64(*client_micros).u64(*server_micros);
        }
        ServerMessage::WriteReply(reply) => {
            header(&mut w, kind::WRITE_REPLY, request_id);
            w.u8(u8::from(reply.accepted)).string(&reply.reason);
            reply.actual_value.encode(&mut w);
        }
    }
    w.into_vec()
}

/// Decodes a server body. Returns `(requestId, message)`.
pub fn decode_server(body: &[u8]) -> WireResult<(u32, ServerMessage)> {
    let mut r = Reader::new(body);
    let k = r.u8()?;
    let request_id = r.u32()?;
    let message = match k {
        kind::REPLY => {
            let status = match r.u8()? {
                0 => Status::Ok,
                1 => Status::Rejected,
                2 => Status::Error,
                _ => return Err(WireError::Invalid("reply status")),
            };
            let reason = r.string()?;
            let data = decode_reply_data(&mut r)?;
            ServerMessage::Reply { status, reason, data }
        }
        kind::SCHEMA => {
            let dictionary = read_dictionary(&mut r)?;
            let n = r.varint()?;
            if n > r.remaining() as u64 {
                return Err(WireError::InvalidLength(n));
            }
            let mut schemas = Vec::with_capacity(n as usize);
            for _ in 0..n {
                schemas.push(read_struct_schema(&mut r)?);
            }
            ServerMessage::Schema { dictionary, schemas }
        }
        kind::TIME_PONG => ServerMessage::TimePong { client_micros: r.u64()?, server_micros: r.u64()? },
        kind::WRITE_REPLY => {
            let accepted = match r.u8()? {
                0 => false,
                1 => true,
                _ => return Err(WireError::Invalid("accepted flag")),
            };
            let reason = r.string()?;
            let actual_value = Value::decode(&mut r)?;
            ServerMessage::WriteReply(WriteReply { accepted, reason, actual_value })
        }
        _ => return Err(WireError::Invalid("unknown server message kind")),
    };
    expect_end(&r)?;
    Ok((request_id, message))
}

/// Prefixes a body with its `u32 LE` length. Fails if the body exceeds [`MAX_CONTROL_BODY`].
pub fn frame(body: &[u8]) -> WireResult<Vec<u8>> {
    if body.len() > MAX_CONTROL_BODY {
        return Err(WireError::InvalidLength(body.len() as u64));
    }
    let mut out = Vec::with_capacity(body.len() + 4);
    out.extend_from_slice(&(body.len() as u32).to_le_bytes());
    out.extend_from_slice(body);
    Ok(out)
}

/// Tries to take one complete framed body from the front of `buf`.
/// Returns `Ok(None)` when more bytes are needed, otherwise `(consumed, body)`.
pub fn take_frame(buf: &[u8]) -> WireResult<Option<(usize, &[u8])>> {
    if buf.len() < 4 {
        return Ok(None);
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    if len > MAX_CONTROL_BODY {
        return Err(WireError::InvalidLength(len as u64));
    }
    if buf.len() < 4 + len {
        return Ok(None);
    }
    Ok(Some((4 + len, &buf[4..4 + len])))
}

fn header(w: &mut Writer, kind: u8, request_id: u32) {
    w.u8(kind).u32(request_id);
}

fn expect_end(r: &Reader<'_>) -> WireResult<()> {
    if r.has_remaining() {
        Err(WireError::Invalid("trailing bytes in control message"))
    } else {
        Ok(())
    }
}

fn encode_reply_data(w: &mut Writer, data: &ReplyData) {
    match data {
        ReplyData::None => {
            w.u8(0);
        }
        ReplyData::Subscription { subscription_id } => {
            w.u8(1).u32(*subscription_id);
        }
        ReplyData::Session { session_id } => {
            w.u8(2).u32(*session_id);
        }
        ReplyData::Files(files) => {
            w.u8(3).varint(files.len() as u64);
            for f in files {
                w.string(&f.name).u64(f.size).u64(f.modified_unix_secs);
            }
        }
        ReplyData::Chunk { eof, data } => {
            w.u8(4).u8(u8::from(*eof)).varint(data.len() as u64).bytes(data);
        }
    }
}

fn decode_reply_data(r: &mut Reader<'_>) -> WireResult<ReplyData> {
    Ok(match r.u8()? {
        0 => ReplyData::None,
        1 => ReplyData::Subscription { subscription_id: r.u32()? },
        2 => ReplyData::Session { session_id: r.u32()? },
        3 => {
            let raw = r.varint()?;
            let n = usize::try_from(raw).map_err(|_| WireError::InvalidLength(raw))?;
            if n > r.remaining() {
                return Err(WireError::InvalidLength(raw));
            }
            let mut files = Vec::with_capacity(n);
            for _ in 0..n {
                files.push(FileInfo { name: r.string()?, size: r.u64()?, modified_unix_secs: r.u64()? });
            }
            ReplyData::Files(files)
        }
        4 => {
            let eof = match r.u8()? {
                0 => false,
                1 => true,
                _ => return Err(WireError::Invalid("eof flag")),
            };
            let len = r.length()?;
            ReplyData::Chunk { eof, data: r.bytes(len)?.to_vec() }
        }
        _ => return Err(WireError::Invalid("reply data kind")),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v2::record::{DictEntry, DICT_FLAG_WRITABLE};

    #[test]
    fn client_messages_round_trip() {
        let messages = vec![
            ClientMessage::Hello { client_name: "dash".into(), protocol_version: 2, udp_port: 5810 },
            ClientMessage::Subscribe { pattern: "robot/turret/**".into(), rate: Rate::Hz(50.0) },
            ClientMessage::Subscribe { pattern: "robot/**".into(), rate: Rate::OnChange },
            ClientMessage::Unsubscribe { subscription_id: 7 },
            ClientMessage::Write { signal_id: 300, value: Value::F64(1.25) },
            ClientMessage::ActionRun { signal_id: 9 },
            ClientMessage::ActionCancel { signal_id: 9 },
            ClientMessage::TimePing { client_micros: 123_456 },
            ClientMessage::SchemaRequest,
            ClientMessage::FileList,
            ClientMessage::FileDownload { name: "a.athrp".into(), offset: 10, max_len: 4096 },
            ClientMessage::FileDelete { name: "b.json".into() },
        ];
        for (i, msg) in messages.into_iter().enumerate() {
            let body = encode_client(i as u32 + 1, &msg);
            let (id, decoded) = decode_client(&body).unwrap();
            assert_eq!(id, i as u32 + 1);
            assert_eq!(decoded, msg);
        }
    }

    #[test]
    fn server_messages_round_trip() {
        let dictionary = Dictionary {
            entries: vec![DictEntry {
                signal_id: 1,
                path: "robot/turret/angle".into(),
                type_tag: 3,
                unit: "deg".into(),
                role: 1,
                flags: DICT_FLAG_WRITABLE,
                lock_group: "InMatch".into(),
                struct_schema_id: 0,
            }],
        };
        let schemas = vec![StructSchema {
            id: 1,
            type_name: "Pose2d".into(),
            schema: "Translation2d translation;Rotation2d rotation".into(),
        }];
        let messages = vec![
            ServerMessage::Reply { status: Status::Rejected, reason: "rejected: locked (InMatch)".into(), data: ReplyData::None },
            ServerMessage::Reply { status: Status::Ok, reason: String::new(), data: ReplyData::Subscription { subscription_id: 4 } },
            ServerMessage::Reply { status: Status::Ok, reason: String::new(), data: ReplyData::Session { session_id: 2 } },
            ServerMessage::Reply {
                status: Status::Ok,
                reason: String::new(),
                data: ReplyData::Files(vec![FileInfo { name: "x.athrp".into(), size: 99, modified_unix_secs: 5 }]),
            },
            ServerMessage::Reply { status: Status::Ok, reason: String::new(), data: ReplyData::Chunk { eof: true, data: vec![1, 2, 3] } },
            ServerMessage::Schema { dictionary, schemas },
            ServerMessage::TimePong { client_micros: 1, server_micros: 2 },
            ServerMessage::WriteReply(WriteReply::rejected("rejected: locked (InMatch)", Value::F64(1.5))),
        ];
        for msg in messages {
            let body = encode_server(42, &msg);
            let (id, decoded) = decode_server(&body).unwrap();
            assert_eq!(id, 42);
            assert_eq!(decoded, msg);
        }
    }

    #[test]
    fn framing_waits_for_complete_bodies() {
        let body = encode_client(1, &ClientMessage::FileList);
        let framed = frame(&body).unwrap();
        assert!(take_frame(&framed[..framed.len() - 1]).unwrap().is_none());
        let (consumed, got) = take_frame(&framed).unwrap().unwrap();
        assert_eq!(consumed, framed.len());
        assert_eq!(got, body.as_slice());
    }

    #[test]
    fn rate_validation() {
        assert!(Rate::Hz(0.0).validate().is_err());
        assert!(Rate::Hz(f64::NAN).validate().is_err());
        assert!(Rate::Hz(50.0).validate().is_ok());
        assert!(Rate::OnChange.validate().is_ok());
    }
}
