//! ARCP v2 wire protocol: byte-exact codec for records, frames, markers, metadata, struct
//! schemas (appendix E1-W), the control channel (E4) and mechanism-path patterns (E2).
//!
//! The Java reference implementation is `athena-runtime` `ca.frc6390.athena.runtime.wire`; the
//! shared golden vectors live in `docs/wire-vectors/`.

pub mod control;
pub mod pattern;
pub mod record;
pub mod value;
pub mod wire;

pub use control::{
    decode_client, decode_server, encode_client, encode_server, frame, take_frame, ClientMessage,
    FileInfo, Rate, ReplyData, ServerMessage, Status, CONTROL_PROTOCOL_VERSION, MAX_FILE_CHUNK,
};
pub use record::{
    decode_compressed_payload, decode_replay, decode_stream, encode_compressed_block, encode_stream,
    replay_header, DictEntry, Dictionary, Entry, Frame, Marker, Metadata, Record, StructSchema,
};
pub use value::Value;
pub use wire::{Reader, WireError, WireResult, Writer};
