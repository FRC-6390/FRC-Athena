//! Record envelopes, record payloads and replay files (appendix E1-W).
//!
//! Byte layout is shared with the Java reference codec and with the golden vectors in
//! `docs/wire-vectors/`. Unknown record types are preserved as [`Record::Unknown`] so that
//! re-encoding a stream is always byte-identical.

use std::io::Read;

use flate2::read::DeflateDecoder;
use flate2::write::DeflateEncoder;
use flate2::Compression;
use std::io::Write as _;

use super::value::Value;
use super::wire::{Reader, WireError, WireResult, Writer};

pub const RECORD_DICTIONARY: u8 = 0x01;
pub const RECORD_FRAME: u8 = 0x02;
pub const RECORD_MARKER: u8 = 0x03;
pub const RECORD_METADATA: u8 = 0x04;
pub const RECORD_STRUCT_SCHEMA: u8 = 0x05;
pub const RECORD_COMPRESSED_BLOCK: u8 = 0x10;

pub const FRAME_VERSION: u8 = 2;
pub const FRAME_FLAG_KEYFRAME: u8 = 0x01;
pub const FRAME_FLAG_FRAGMENT: u8 = 0x02;

pub const DICT_FLAG_WRITABLE: u8 = 0x01;
pub const DICT_FLAG_PERSISTED: u8 = 0x02;

pub const REPLAY_MAGIC: &[u8; 6] = b"ATHRP\0";
pub const REPLAY_FORMAT_VERSION: u16 = 1;

/// Marker kinds from appendix E1-W.
pub mod marker_kind {
    pub const BOOT: u64 = 1;
    pub const MATCH_IDENTIFIED: u64 = 2;
    pub const AUTO_START: u64 = 3;
    pub const TELEOP_START: u64 = 4;
    pub const MATCH_END: u64 = 5;
    pub const DISABLE: u64 = 6;
    pub const FAULT: u64 = 7;
    pub const STAGE_CHANGE: u64 = 8;
    pub const CHECK: u64 = 9;
    pub const OVERRIDE: u64 = 10;
    pub const PERSISTED_WRITE: u64 = 11;
    pub const RECORD: u64 = 12;
    pub const TEST_START: u64 = 13;
    pub const USER_BASE: u64 = 100;
}

/// Signal roles used in the dictionary (normative values from the off-season spec).
pub mod role {
    pub const UNSPECIFIED: u8 = 0;
    pub const STATE: u8 = 1;
    pub const VALUE: u8 = 2;
    pub const CONTROL: u8 = 3;
    pub const DEVICE: u8 = 4;
    pub const ACTION: u8 = 5;
    pub const HOOK: u8 = 6;
    /// Boolean lock-state signal at `athena/locks/<group>` (true = locked).
    pub const LOCK: u8 = 7;
    pub const SYSTEM: u8 = 8;
}

/// One dictionary entry (signal announcement).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DictEntry {
    pub signal_id: u64,
    pub path: String,
    pub type_tag: u8,
    /// Empty when the signal has no unit.
    pub unit: String,
    pub role: u8,
    /// Bit 0 writable, bit 1 persisted. Unknown bits are preserved.
    pub flags: u8,
    /// Empty when the signal has no lock group.
    pub lock_group: String,
    /// 0 when the signal is not a struct.
    pub struct_schema_id: u64,
}

impl DictEntry {
    pub fn writable(&self) -> bool {
        self.flags & DICT_FLAG_WRITABLE != 0
    }

    pub fn persisted(&self) -> bool {
        self.flags & DICT_FLAG_PERSISTED != 0
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Dictionary {
    pub entries: Vec<DictEntry>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StructSchema {
    pub id: u64,
    pub type_name: String,
    /// WPILib struct schema string, for example `"Translation2d translation;Rotation2d rotation"`.
    pub schema: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Entry {
    pub signal_id: u64,
    pub value: Value,
}

/// One frame (one robot cycle, or one fragment of it).
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// Raw flags byte. Bit 0 keyframe, bit 1 fragment; unknown bits are preserved.
    pub flags: u8,
    pub timestamp_micros: u64,
    pub sequence: u32,
    pub fragment_index: u16,
    pub fragment_count: u16,
    pub entries: Vec<Entry>,
}

impl Frame {
    /// A non-fragmented frame.
    pub fn new(keyframe: bool, timestamp_micros: u64, sequence: u32, entries: Vec<Entry>) -> Self {
        Self {
            flags: if keyframe { FRAME_FLAG_KEYFRAME } else { 0 },
            timestamp_micros,
            sequence,
            fragment_index: 0,
            fragment_count: 1,
            entries,
        }
    }

    pub fn keyframe(&self) -> bool {
        self.flags & FRAME_FLAG_KEYFRAME != 0
    }

    pub fn fragmented(&self) -> bool {
        self.flags & FRAME_FLAG_FRAGMENT != 0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Marker {
    pub timestamp_micros: i64,
    pub kind: u64,
    pub message: String,
    /// 0 when the marker is not tied to a signal.
    pub signal_id: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Metadata {
    pub pairs: Vec<(String, String)>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Record {
    Dictionary(Dictionary),
    Frame(Frame),
    Marker(Marker),
    Metadata(Metadata),
    StructSchema(StructSchema),
    /// A record type this codec does not know. Kept verbatim.
    Unknown { record_type: u8, payload: Vec<u8> },
}

impl Record {
    pub fn record_type(&self) -> u8 {
        match self {
            Record::Dictionary(_) => RECORD_DICTIONARY,
            Record::Frame(_) => RECORD_FRAME,
            Record::Marker(_) => RECORD_MARKER,
            Record::Metadata(_) => RECORD_METADATA,
            Record::StructSchema(_) => RECORD_STRUCT_SCHEMA,
            Record::Unknown { record_type, .. } => *record_type,
        }
    }

    /// Writes the full envelope (`type`, `varint length`, payload).
    pub fn encode(&self, out: &mut Writer) {
        let mut payload = Writer::new();
        self.encode_payload(&mut payload);
        out.record(self.record_type(), payload.as_slice());
    }

    fn encode_payload(&self, w: &mut Writer) {
        match self {
            Record::Dictionary(dict) => write_dictionary(w, dict),
            Record::Frame(frame) => {
                w.u8(FRAME_VERSION)
                    .u8(frame.flags)
                    .u64(frame.timestamp_micros)
                    .u32(frame.sequence)
                    .u16(frame.fragment_index)
                    .u16(frame.fragment_count)
                    .varint(frame.entries.len() as u64);
                for e in &frame.entries {
                    w.varint(e.signal_id);
                    e.value.encode(w);
                }
            }
            Record::Marker(m) => {
                w.zigzag(m.timestamp_micros)
                    .varint(m.kind)
                    .string(&m.message)
                    .varint(m.signal_id);
            }
            Record::Metadata(meta) => {
                w.varint(meta.pairs.len() as u64);
                for (k, v) in &meta.pairs {
                    w.string(k).string(v);
                }
            }
            Record::StructSchema(s) => write_struct_schema(w, s),
            Record::Unknown { payload, .. } => {
                w.bytes(payload);
            }
        }
    }

    /// Decodes one payload of a non-compressed record type.
    pub fn decode_payload(record_type: u8, payload: &[u8]) -> WireResult<Record> {
        let mut r = Reader::new(payload);
        let record = match record_type {
            RECORD_DICTIONARY => Record::Dictionary(read_dictionary(&mut r)?),
            RECORD_FRAME => {
                let version = r.u8()?;
                if version != FRAME_VERSION {
                    return Err(WireError::Invalid("frame version"));
                }
                let flags = r.u8()?;
                let timestamp_micros = r.u64()?;
                let sequence = r.u32()?;
                let fragment_index = r.u16()?;
                let fragment_count = r.u16()?;
                let n = count(&mut r, 3)?;
                let mut entries = Vec::with_capacity(n);
                for _ in 0..n {
                    let signal_id = r.varint()?;
                    let value = Value::decode(&mut r)?;
                    entries.push(Entry { signal_id, value });
                }
                Record::Frame(Frame { flags, timestamp_micros, sequence, fragment_index, fragment_count, entries })
            }
            RECORD_MARKER => Record::Marker(Marker {
                timestamp_micros: r.zigzag()?,
                kind: r.varint()?,
                message: r.string()?,
                signal_id: r.varint()?,
            }),
            RECORD_METADATA => {
                let n = count(&mut r, 2)?;
                let mut pairs = Vec::with_capacity(n);
                for _ in 0..n {
                    pairs.push((r.string()?, r.string()?));
                }
                Record::Metadata(Metadata { pairs })
            }
            RECORD_STRUCT_SCHEMA => Record::StructSchema(read_struct_schema(&mut r)?),
            RECORD_COMPRESSED_BLOCK => {
                return Err(WireError::Invalid("compressed block must be expanded by decode_stream"))
            }
            other => {
                return Ok(Record::Unknown { record_type: other, payload: payload.to_vec() });
            }
        };
        if r.has_remaining() {
            return Err(WireError::Invalid("trailing bytes in record payload"));
        }
        Ok(record)
    }
}

/// Writes a dictionary body: `varint count`, then each entry.
pub fn write_dictionary(w: &mut Writer, dict: &Dictionary) {
    w.varint(dict.entries.len() as u64);
    for e in &dict.entries {
        w.varint(e.signal_id)
            .string(&e.path)
            .u8(e.type_tag)
            .string(&e.unit)
            .u8(e.role)
            .u8(e.flags)
            .string(&e.lock_group)
            .varint(e.struct_schema_id);
    }
}

/// Reads a dictionary body written by [`write_dictionary`].
pub fn read_dictionary(r: &mut Reader<'_>) -> WireResult<Dictionary> {
    let n = count(r, 8)?;
    let mut entries = Vec::with_capacity(n);
    for _ in 0..n {
        entries.push(DictEntry {
            signal_id: r.varint()?,
            path: r.string()?,
            type_tag: r.u8()?,
            unit: r.string()?,
            role: r.u8()?,
            flags: r.u8()?,
            lock_group: r.string()?,
            struct_schema_id: r.varint()?,
        });
    }
    Ok(Dictionary { entries })
}

/// Writes a struct schema body: `varint id`, string type name, string schema.
pub fn write_struct_schema(w: &mut Writer, s: &StructSchema) {
    w.varint(s.id).string(&s.type_name).string(&s.schema);
}

/// Reads a struct schema body written by [`write_struct_schema`].
pub fn read_struct_schema(r: &mut Reader<'_>) -> WireResult<StructSchema> {
    Ok(StructSchema { id: r.varint()?, type_name: r.string()?, schema: r.string()? })
}

/// Reads an element count and rejects counts that cannot fit (each element needs `min_bytes`).
fn count(r: &mut Reader<'_>, min_bytes: usize) -> WireResult<usize> {
    let raw = r.varint()?;
    let n = usize::try_from(raw).map_err(|_| WireError::InvalidLength(raw))?;
    if n.saturating_mul(min_bytes) > r.remaining() {
        return Err(WireError::InvalidLength(raw));
    }
    Ok(n)
}

/// Encodes a list of records as a plain (uncompressed) record stream.
pub fn encode_stream(records: &[Record]) -> Vec<u8> {
    let mut w = Writer::new();
    for record in records {
        record.encode(&mut w);
    }
    w.into_vec()
}

/// Encodes records as one `COMPRESSED_BLOCK` envelope: `varint uncompressedLength` followed by
/// raw DEFLATE of the inner record stream.
pub fn encode_compressed_block(records: &[Record]) -> Vec<u8> {
    let inner = encode_stream(records);
    let mut encoder = DeflateEncoder::new(Vec::new(), Compression::default());
    // Writing to a Vec cannot fail.
    encoder.write_all(&inner).expect("deflate into Vec");
    let deflated = encoder.finish().expect("deflate into Vec");
    let mut payload = Writer::with_capacity(deflated.len() + 10);
    payload.varint(inner.len() as u64).bytes(&deflated);
    let mut out = Writer::new();
    out.record(RECORD_COMPRESSED_BLOCK, payload.as_slice());
    out.into_vec()
}

/// Decodes a record stream, expanding `COMPRESSED_BLOCK` records in place.
/// Nested compressed blocks are rejected.
pub fn decode_stream(data: &[u8]) -> WireResult<Vec<Record>> {
    let mut out = Vec::new();
    let mut r = Reader::new(data);
    while r.has_remaining() {
        let (record_type, payload) = r.record()?;
        if record_type == RECORD_COMPRESSED_BLOCK {
            out.extend(decode_compressed_payload(payload)?);
        } else {
            out.push(Record::decode_payload(record_type, payload)?);
        }
    }
    Ok(out)
}

/// Decodes the payload of a `COMPRESSED_BLOCK` record into its inner records.
pub fn decode_compressed_payload(payload: &[u8]) -> WireResult<Vec<Record>> {
    let mut r = Reader::new(payload);
    let declared = r.varint()?;
    let declared_len = usize::try_from(declared).map_err(|_| WireError::InvalidLength(declared))?;
    let deflated = r.bytes(r.remaining())?;
    let mut inflated = Vec::with_capacity(declared_len.min(1 << 20));
    DeflateDecoder::new(deflated)
        .take(declared + 1)
        .read_to_end(&mut inflated)
        .map_err(|_| WireError::Invalid("corrupt deflate data"))?;
    if inflated.len() != declared_len {
        return Err(WireError::Invalid("compressed block length mismatch"));
    }
    let mut inner = Vec::new();
    let mut ir = Reader::new(&inflated);
    while ir.has_remaining() {
        let (record_type, body) = ir.record()?;
        if record_type == RECORD_COMPRESSED_BLOCK {
            return Err(WireError::Invalid("nested compressed block"));
        }
        inner.push(Record::decode_payload(record_type, body)?);
    }
    Ok(inner)
}

/// Replay file header: `ATHRP\0` followed by `u16` format version (little-endian).
pub fn replay_header() -> Vec<u8> {
    let mut w = Writer::new();
    w.bytes(REPLAY_MAGIC).u16(REPLAY_FORMAT_VERSION);
    w.into_vec()
}

/// Decodes a complete `.athrp` file (header plus record stream).
pub fn decode_replay(data: &[u8]) -> WireResult<Vec<Record>> {
    let mut r = Reader::new(data);
    if r.bytes(REPLAY_MAGIC.len())? != REPLAY_MAGIC {
        return Err(WireError::Invalid("replay magic"));
    }
    if r.u16()? != REPLAY_FORMAT_VERSION {
        return Err(WireError::Invalid("replay format version"));
    }
    decode_stream(&data[r.position()..])
}
