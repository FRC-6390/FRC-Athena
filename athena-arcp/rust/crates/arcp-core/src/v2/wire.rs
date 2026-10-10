//! Little-endian primitive reader/writer for the ARCP v2 / replay format (appendix E1-W).
//!
//! Mirrors `ca.frc6390.athena.runtime.wire.WireReader` / `WireWriter` byte for byte.

use std::fmt;

/// Decoding failure. Encoding never fails.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireError {
    /// Input ended before the field was complete.
    Truncated,
    /// A varint used more than 10 bytes.
    VarintTooLong,
    /// A length or count does not fit the remaining input or the platform.
    InvalidLength(u64),
    /// A string field was not valid UTF-8.
    InvalidUtf8,
    /// Structural problem with a specific field.
    Invalid(&'static str),
}

impl fmt::Display for WireError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WireError::Truncated => write!(f, "truncated wire data"),
            WireError::VarintTooLong => write!(f, "varint is too long"),
            WireError::InvalidLength(len) => write!(f, "invalid wire length {len}"),
            WireError::InvalidUtf8 => write!(f, "string is not valid UTF-8"),
            WireError::Invalid(what) => write!(f, "invalid wire data: {what}"),
        }
    }
}

impl std::error::Error for WireError {}

pub type WireResult<T> = Result<T, WireError>;

/// Growable little-endian writer.
#[derive(Debug, Default, Clone)]
pub struct Writer {
    buf: Vec<u8>,
}

impl Writer {
    pub fn new() -> Self {
        Self { buf: Vec::new() }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self { buf: Vec::with_capacity(capacity) }
    }

    pub fn len(&self) -> usize {
        self.buf.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    pub fn clear(&mut self) {
        self.buf.clear();
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buf
    }

    pub fn into_vec(self) -> Vec<u8> {
        self.buf
    }

    pub fn u8(&mut self, value: u8) -> &mut Self {
        self.buf.push(value);
        self
    }

    pub fn u16(&mut self, value: u16) -> &mut Self {
        self.buf.extend_from_slice(&value.to_le_bytes());
        self
    }

    pub fn u32(&mut self, value: u32) -> &mut Self {
        self.buf.extend_from_slice(&value.to_le_bytes());
        self
    }

    pub fn u64(&mut self, value: u64) -> &mut Self {
        self.buf.extend_from_slice(&value.to_le_bytes());
        self
    }

    pub fn f64(&mut self, value: f64) -> &mut Self {
        self.u64(value.to_bits())
    }

    /// Unsigned LEB128.
    pub fn varint(&mut self, mut value: u64) -> &mut Self {
        while value & !0x7F != 0 {
            self.buf.push(((value & 0x7F) as u8) | 0x80);
            value >>= 7;
        }
        self.buf.push(value as u8);
        self
    }

    /// Signed zigzag followed by unsigned LEB128.
    pub fn zigzag(&mut self, value: i64) -> &mut Self {
        self.varint(((value << 1) ^ (value >> 63)) as u64)
    }

    /// varint byte length followed by UTF-8 bytes.
    pub fn string(&mut self, value: &str) -> &mut Self {
        self.varint(value.len() as u64);
        self.buf.extend_from_slice(value.as_bytes());
        self
    }

    pub fn bytes(&mut self, value: &[u8]) -> &mut Self {
        self.buf.extend_from_slice(value);
        self
    }

    /// Appends a record envelope: `u8 type`, `varint length`, payload bytes.
    pub fn record(&mut self, record_type: u8, payload: &[u8]) -> &mut Self {
        self.u8(record_type);
        self.varint(payload.len() as u64);
        self.buf.extend_from_slice(payload);
        self
    }
}

/// Bounds-checked little-endian reader over a byte slice.
#[derive(Debug, Clone)]
pub struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    pub fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    pub fn has_remaining(&self) -> bool {
        self.pos < self.data.len()
    }

    pub fn position(&self) -> usize {
        self.pos
    }

    fn take(&mut self, count: usize) -> WireResult<&'a [u8]> {
        if count > self.remaining() {
            return Err(WireError::Truncated);
        }
        let slice = &self.data[self.pos..self.pos + count];
        self.pos += count;
        Ok(slice)
    }

    pub fn u8(&mut self) -> WireResult<u8> {
        Ok(self.take(1)?[0])
    }

    pub fn u16(&mut self) -> WireResult<u16> {
        let b = self.take(2)?;
        Ok(u16::from_le_bytes([b[0], b[1]]))
    }

    pub fn u32(&mut self) -> WireResult<u32> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    pub fn u64(&mut self) -> WireResult<u64> {
        let b = self.take(8)?;
        let mut arr = [0u8; 8];
        arr.copy_from_slice(b);
        Ok(u64::from_le_bytes(arr))
    }

    pub fn f64(&mut self) -> WireResult<f64> {
        Ok(f64::from_bits(self.u64()?))
    }

    pub fn varint(&mut self) -> WireResult<u64> {
        let mut value: u64 = 0;
        let mut shift = 0u32;
        loop {
            if shift > 63 {
                return Err(WireError::VarintTooLong);
            }
            let next = self.u8()?;
            value |= u64::from(next & 0x7F) << shift;
            if next & 0x80 == 0 {
                return Ok(value);
            }
            shift += 7;
        }
    }

    pub fn zigzag(&mut self) -> WireResult<i64> {
        let raw = self.varint()?;
        Ok(((raw >> 1) as i64) ^ -((raw & 1) as i64))
    }

    /// Reads a varint length and checks it against the remaining input.
    pub fn length(&mut self) -> WireResult<usize> {
        let raw = self.varint()?;
        usize::try_from(raw)
            .ok()
            .filter(|len| *len <= self.remaining())
            .ok_or(WireError::InvalidLength(raw))
    }

    pub fn string(&mut self) -> WireResult<String> {
        let len = self.length()?;
        let bytes = self.take(len)?;
        std::str::from_utf8(bytes)
            .map(str::to_owned)
            .map_err(|_| WireError::InvalidUtf8)
    }

    pub fn bytes(&mut self, count: usize) -> WireResult<&'a [u8]> {
        self.take(count)
    }

    /// Reads one record envelope and returns its type and payload slice.
    pub fn record(&mut self) -> WireResult<(u8, &'a [u8])> {
        let record_type = self.u8()?;
        let len = self.length()?;
        Ok((record_type, self.take(len)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn varint_round_trips_edge_values() {
        for value in [0u64, 1, 127, 128, 300, 16_383, 16_384, u32::MAX as u64, u64::MAX] {
            let mut w = Writer::new();
            w.varint(value);
            let mut r = Reader::new(w.as_slice());
            assert_eq!(r.varint().unwrap(), value);
            assert!(!r.has_remaining());
        }
    }

    #[test]
    fn zigzag_matches_reference_encoding() {
        // Reference: zigzag(-1) = 1, zigzag(1) = 2, zigzag(-5000) = 9999.
        let mut w = Writer::new();
        w.zigzag(-1).zigzag(1).zigzag(-5000);
        assert_eq!(w.as_slice(), &[0x01, 0x02, 0x8F, 0x4E]);
        let mut r = Reader::new(w.as_slice());
        assert_eq!(r.zigzag().unwrap(), -1);
        assert_eq!(r.zigzag().unwrap(), 1);
        assert_eq!(r.zigzag().unwrap(), -5000);
    }

    #[test]
    fn overlong_varint_is_rejected() {
        let bytes = [0x80u8; 11];
        assert_eq!(Reader::new(&bytes).varint(), Err(WireError::VarintTooLong));
    }

    #[test]
    fn length_prefix_past_end_is_rejected() {
        let mut w = Writer::new();
        w.varint(10).u8(1);
        assert_eq!(Reader::new(w.as_slice()).string(), Err(WireError::InvalidLength(10)));
    }
}
