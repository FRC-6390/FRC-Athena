//! JNI marshalling format (not the ARCP wire format). Java writes these buffers in
//! `ca.frc6390.athena.arcp.ArcpMarshal`; Rust decodes them into `arcp-core` types and the real
//! ARCP encoding happens inside `arcp-server`. All integers are little-endian.
//!
//! Value (also used for requests and replies): `u8 kind` then a body. `kind` equals the ARCP
//! `typeTag` (1 bool, 2 i64, 3 f64, 4 string, 5 bool[], 6 i64[], 7 f64[], 8 string[], 9 struct).
//! Bodies: bool `u8`; i64 `i64`; f64 `f64`; string `u32 len, utf8`; bool[] `u32 n, n*u8`;
//! i64[] `u32 n, n*i64`; f64[] `u32 n, n*f64`; string[] `u32 n, n*(u32 len, utf8)`;
//! struct `u64 schemaId, u32 len, bytes`.
//!
//! Cycle batch: `u32 count, count*(u64 signalId, value)`.
//!
//! Dictionary: `u32 count, count*(u64 signalId, u8 typeTag, u8 role, u8 flags, u64 structSchemaId,
//! str path, str unit, str lockGroup)`, then `u32 schemaCount, schemaCount*(u64 id, str typeName,
//! str schema)`. `str` = `u32 len, utf8`.
//!
//! Requests (Rust to Java): `u32 count, count*(u64 requestId, u8 op, u64 signalId, [value if op ==
//! WRITE])`. Replies (Java to Rust): `u32 count, count*(u64 requestId, u8 op, u8 accepted, str
//! reason, [value if op == WRITE])`.

use arcp_core::v2::record::{DictEntry, Dictionary, StructSchema};
use arcp_core::v2::{Entry, Value};

pub const KIND_BOOL: u8 = 1;
pub const KIND_I64: u8 = 2;
pub const KIND_F64: u8 = 3;
pub const KIND_STRING: u8 = 4;
pub const KIND_BOOL_ARRAY: u8 = 5;
pub const KIND_I64_ARRAY: u8 = 6;
pub const KIND_F64_ARRAY: u8 = 7;
pub const KIND_STRING_ARRAY: u8 = 8;
pub const KIND_STRUCT: u8 = 9;

/// Refuses absurd lengths before allocating (protects against a corrupt buffer).
const MAX_ITEMS: usize = 1 << 20;

pub struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], String> {
        let end = self.pos.checked_add(n).filter(|&e| e <= self.data.len());
        match end {
            Some(end) => {
                let slice = &self.data[self.pos..end];
                self.pos = end;
                Ok(slice)
            }
            None => Err(format!(
                "marshal buffer truncated at {} (need {n} bytes)",
                self.pos
            )),
        }
    }

    pub fn u8(&mut self) -> Result<u8, String> {
        Ok(self.take(1)?[0])
    }

    pub fn u32(&mut self) -> Result<u32, String> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    pub fn u64(&mut self) -> Result<u64, String> {
        let b = self.take(8)?;
        let mut a = [0u8; 8];
        a.copy_from_slice(b);
        Ok(u64::from_le_bytes(a))
    }

    fn count(&mut self) -> Result<usize, String> {
        let n = self.u32()? as usize;
        if n > MAX_ITEMS {
            return Err(format!("marshal count {n} exceeds limit"));
        }
        Ok(n)
    }

    /// Reads `n` raw bytes.
    pub fn bytes(&mut self, n: usize) -> Result<&'a [u8], String> {
        self.take(n)
    }

    pub fn string(&mut self) -> Result<String, String> {
        let n = self.u32()? as usize;
        let bytes = self.take(n)?;
        std::str::from_utf8(bytes)
            .map(str::to_owned)
            .map_err(|_| "marshal string is not UTF-8".to_owned())
    }

    pub fn value(&mut self) -> Result<Value, String> {
        let kind = self.u8()?;
        Ok(match kind {
            KIND_BOOL => Value::Bool(self.u8()? != 0),
            KIND_I64 => Value::I64(self.u64()? as i64),
            KIND_F64 => Value::F64(f64::from_bits(self.u64()?)),
            KIND_STRING => Value::Str(self.string()?),
            KIND_BOOL_ARRAY => {
                let n = self.count()?;
                Value::BoolArray(self.take(n)?.iter().map(|&b| b != 0).collect())
            }
            KIND_I64_ARRAY => {
                let n = self.count()?;
                let mut v = Vec::with_capacity(n);
                for _ in 0..n {
                    v.push(self.u64()? as i64);
                }
                Value::I64Array(v)
            }
            KIND_F64_ARRAY => {
                let n = self.count()?;
                let mut v = Vec::with_capacity(n);
                for _ in 0..n {
                    v.push(f64::from_bits(self.u64()?));
                }
                Value::F64Array(v)
            }
            KIND_STRING_ARRAY => {
                let n = self.count()?;
                let mut v = Vec::with_capacity(n);
                for _ in 0..n {
                    v.push(self.string()?);
                }
                Value::StrArray(v)
            }
            KIND_STRUCT => {
                let schema_id = self.u64()?;
                let n = self.u32()? as usize;
                Value::Struct {
                    schema_id,
                    bytes: self.take(n)?.to_vec(),
                }
            }
            other => return Err(format!("unknown marshal value kind {other}")),
        })
    }

    pub fn cycle(&mut self) -> Result<Vec<Entry>, String> {
        let n = self.count()?;
        let mut out = Vec::with_capacity(n);
        for _ in 0..n {
            let signal_id = self.u64()?;
            out.push(Entry {
                signal_id,
                value: self.value()?,
            });
        }
        Ok(out)
    }

    pub fn dictionary(&mut self) -> Result<(Dictionary, Vec<StructSchema>), String> {
        let n = self.count()?;
        let mut entries = Vec::with_capacity(n);
        for _ in 0..n {
            entries.push(DictEntry {
                signal_id: self.u64()?,
                type_tag: self.u8()?,
                role: self.u8()?,
                flags: self.u8()?,
                struct_schema_id: self.u64()?,
                path: self.string()?,
                unit: self.string()?,
                lock_group: self.string()?,
            });
        }
        let s = self.count()?;
        let mut schemas = Vec::with_capacity(s);
        for _ in 0..s {
            schemas.push(StructSchema {
                id: self.u64()?,
                type_name: self.string()?,
                schema: self.string()?,
            });
        }
        Ok((Dictionary { entries }, schemas))
    }

    pub fn finished(&self) -> bool {
        self.pos == self.data.len()
    }
}

pub fn put_u8(out: &mut Vec<u8>, v: u8) {
    out.push(v);
}

pub fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

pub fn put_u64(out: &mut Vec<u8>, v: u64) {
    out.extend_from_slice(&v.to_le_bytes());
}

pub fn put_string(out: &mut Vec<u8>, s: &str) {
    put_u32(out, s.len() as u32);
    out.extend_from_slice(s.as_bytes());
}

pub fn put_value(out: &mut Vec<u8>, value: &Value) {
    match value {
        Value::Bool(b) => {
            put_u8(out, KIND_BOOL);
            put_u8(out, u8::from(*b));
        }
        Value::I64(v) => {
            put_u8(out, KIND_I64);
            put_u64(out, *v as u64);
        }
        Value::F64(v) => {
            put_u8(out, KIND_F64);
            put_u64(out, v.to_bits());
        }
        Value::Str(s) => {
            put_u8(out, KIND_STRING);
            put_string(out, s);
        }
        Value::BoolArray(v) => {
            put_u8(out, KIND_BOOL_ARRAY);
            put_u32(out, v.len() as u32);
            out.extend(v.iter().map(|&b| u8::from(b)));
        }
        Value::I64Array(v) => {
            put_u8(out, KIND_I64_ARRAY);
            put_u32(out, v.len() as u32);
            for x in v {
                put_u64(out, *x as u64);
            }
        }
        Value::F64Array(v) => {
            put_u8(out, KIND_F64_ARRAY);
            put_u32(out, v.len() as u32);
            for x in v {
                put_u64(out, x.to_bits());
            }
        }
        Value::StrArray(v) => {
            put_u8(out, KIND_STRING_ARRAY);
            put_u32(out, v.len() as u32);
            for s in v {
                put_string(out, s);
            }
        }
        Value::Struct { schema_id, bytes } => {
            put_u8(out, KIND_STRUCT);
            put_u64(out, *schema_id);
            put_u32(out, bytes.len() as u32);
            out.extend_from_slice(bytes);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn values_round_trip() {
        let values = vec![
            Value::Bool(true),
            Value::I64(-7),
            Value::F64(2.5),
            Value::Str("turret".into()),
            Value::BoolArray(vec![true, false]),
            Value::I64Array(vec![1, -2]),
            Value::F64Array(vec![0.25]),
            Value::StrArray(vec!["a".into(), "bc".into()]),
            Value::Struct {
                schema_id: 4,
                bytes: vec![9, 8, 7],
            },
        ];
        let mut out = Vec::new();
        for v in &values {
            put_value(&mut out, v);
        }
        let mut r = Reader::new(&out);
        for v in &values {
            assert_eq!(&r.value().unwrap(), v);
        }
        assert!(r.finished());
    }

    #[test]
    fn truncated_buffers_are_errors() {
        let mut out = Vec::new();
        put_value(&mut out, &Value::Str("hello".into()));
        out.truncate(out.len() - 1);
        assert!(Reader::new(&out).value().is_err());
        assert!(Reader::new(&[0xFF]).value().is_err());
    }
}
