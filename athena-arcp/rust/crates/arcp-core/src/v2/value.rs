//! Typed signal values and their `typeTag` encodings (appendix E1-W, "typeTag values").

use super::wire::{Reader, WireError, WireResult, Writer};

pub const TYPE_BOOL: u8 = 1;
pub const TYPE_I64: u8 = 2;
pub const TYPE_F64: u8 = 3;
pub const TYPE_STRING: u8 = 4;
pub const TYPE_BOOL_ARRAY: u8 = 5;
pub const TYPE_I64_ARRAY: u8 = 6;
pub const TYPE_F64_ARRAY: u8 = 7;
pub const TYPE_STRING_ARRAY: u8 = 8;
pub const TYPE_STRUCT: u8 = 9;

/// A typed signal value. Floats are compared by bit pattern through [`Value::same_bits`].
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Bool(bool),
    I64(i64),
    F64(f64),
    Str(String),
    BoolArray(Vec<bool>),
    I64Array(Vec<i64>),
    F64Array(Vec<f64>),
    StrArray(Vec<String>),
    /// WPILib struct bytes plus the id of the `STRUCT_SCHEMA` describing them.
    Struct { schema_id: u64, bytes: Vec<u8> },
}

impl Value {
    /// The `typeTag` used when this value is written.
    pub fn type_tag(&self) -> u8 {
        match self {
            Value::Bool(_) => TYPE_BOOL,
            Value::I64(_) => TYPE_I64,
            Value::F64(_) => TYPE_F64,
            Value::Str(_) => TYPE_STRING,
            Value::BoolArray(_) => TYPE_BOOL_ARRAY,
            Value::I64Array(_) => TYPE_I64_ARRAY,
            Value::F64Array(_) => TYPE_F64_ARRAY,
            Value::StrArray(_) => TYPE_STRING_ARRAY,
            Value::Struct { .. } => TYPE_STRUCT,
        }
    }

    /// Equality that treats floats by bit pattern, so NaN payloads and `-0.0` round-trip exactly.
    pub fn same_bits(&self, other: &Value) -> bool {
        match (self, other) {
            (Value::F64(a), Value::F64(b)) => a.to_bits() == b.to_bits(),
            (Value::F64Array(a), Value::F64Array(b)) => {
                a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
            }
            _ => self == other,
        }
    }

    /// Writes the value body (without the type tag).
    pub fn encode_body(&self, w: &mut Writer) {
        match self {
            Value::Bool(v) => {
                w.u8(u8::from(*v));
            }
            Value::I64(v) => {
                w.zigzag(*v);
            }
            Value::F64(v) => {
                w.f64(*v);
            }
            Value::Str(v) => {
                w.string(v);
            }
            Value::BoolArray(v) => {
                w.varint(v.len() as u64);
                for b in v {
                    w.u8(u8::from(*b));
                }
            }
            Value::I64Array(v) => {
                w.varint(v.len() as u64);
                for n in v {
                    w.zigzag(*n);
                }
            }
            Value::F64Array(v) => {
                w.varint(v.len() as u64);
                for n in v {
                    w.f64(*n);
                }
            }
            Value::StrArray(v) => {
                w.varint(v.len() as u64);
                for s in v {
                    w.string(s);
                }
            }
            Value::Struct { schema_id, bytes } => {
                w.varint(*schema_id);
                w.varint(bytes.len() as u64);
                w.bytes(bytes);
            }
        }
    }

    /// Writes `u8 typeTag` followed by the body.
    pub fn encode(&self, w: &mut Writer) {
        w.u8(self.type_tag());
        self.encode_body(w);
    }

    /// Reads a body for a known `tag`.
    pub fn decode_body(tag: u8, r: &mut Reader<'_>) -> WireResult<Value> {
        Ok(match tag {
            TYPE_BOOL => Value::Bool(match r.u8()? {
                0 => false,
                1 => true,
                _ => return Err(WireError::Invalid("bool value")),
            }),
            TYPE_I64 => Value::I64(r.zigzag()?),
            TYPE_F64 => Value::F64(r.f64()?),
            TYPE_STRING => Value::Str(r.string()?),
            TYPE_BOOL_ARRAY => {
                let n = count(r, 1)?;
                let mut out = Vec::with_capacity(n);
                for _ in 0..n {
                    out.push(match r.u8()? {
                        0 => false,
                        1 => true,
                        _ => return Err(WireError::Invalid("bool array element")),
                    });
                }
                Value::BoolArray(out)
            }
            TYPE_I64_ARRAY => {
                let n = count(r, 1)?;
                let mut out = Vec::with_capacity(n);
                for _ in 0..n {
                    out.push(r.zigzag()?);
                }
                Value::I64Array(out)
            }
            TYPE_F64_ARRAY => {
                let n = count(r, 8)?;
                let mut out = Vec::with_capacity(n);
                for _ in 0..n {
                    out.push(r.f64()?);
                }
                Value::F64Array(out)
            }
            TYPE_STRING_ARRAY => {
                let n = count(r, 1)?;
                let mut out = Vec::with_capacity(n);
                for _ in 0..n {
                    out.push(r.string()?);
                }
                Value::StrArray(out)
            }
            TYPE_STRUCT => {
                let schema_id = r.varint()?;
                let len = r.length()?;
                Value::Struct { schema_id, bytes: r.bytes(len)?.to_vec() }
            }
            _ => return Err(WireError::Invalid("unknown type tag")),
        })
    }

    /// Reads `u8 typeTag` followed by the body.
    pub fn decode(r: &mut Reader<'_>) -> WireResult<Value> {
        let tag = r.u8()?;
        Self::decode_body(tag, r)
    }
}

/// Reads an element count and rejects counts that cannot fit in the remaining input
/// (each element needs at least `min_bytes`).
fn count(r: &mut Reader<'_>, min_bytes: usize) -> WireResult<usize> {
    let raw = r.varint()?;
    let n = usize::try_from(raw).map_err(|_| WireError::InvalidLength(raw))?;
    if n.saturating_mul(min_bytes) > r.remaining() {
        return Err(WireError::InvalidLength(raw));
    }
    Ok(n)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_tag_round_trips() {
        let values = vec![
            Value::Bool(true),
            Value::I64(-1234567890123),
            Value::F64(-0.0),
            Value::Str("hello \u{2713}".into()),
            Value::BoolArray(vec![true, false, true]),
            Value::I64Array(vec![0, -1, i64::MAX]),
            Value::F64Array(vec![1.5, -2.25]),
            Value::StrArray(vec!["a".into(), String::new(), "bc".into()]),
            Value::Struct { schema_id: 1, bytes: vec![0; 24] },
        ];
        for value in values {
            let mut w = Writer::new();
            value.encode(&mut w);
            let mut r = Reader::new(w.as_slice());
            let decoded = Value::decode(&mut r).unwrap();
            assert!(decoded.same_bits(&value), "{value:?}");
            assert!(!r.has_remaining());
        }
    }

    #[test]
    fn bool_byte_must_be_zero_or_one() {
        let bytes = [TYPE_BOOL, 2];
        assert!(Value::decode(&mut Reader::new(&bytes)).is_err());
    }

    #[test]
    fn oversized_array_count_is_rejected_without_allocating() {
        let mut w = Writer::new();
        w.u8(TYPE_F64_ARRAY).varint(1 << 40);
        assert!(Value::decode(&mut Reader::new(w.as_slice())).is_err());
    }
}
