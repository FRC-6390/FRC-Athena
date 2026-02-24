use std::convert::TryFrom;

use crate::constants::{ARCP_PROTOCOL_VERSION, EVENT_ACTION, EVENT_TUNABLE_SET};
use crate::{RuntimeEvent, SignalType, SignalValue};

pub fn encode_update(signal_id: u16, value: &SignalValue) -> Result<Vec<u8>, &'static str> {
    let mut out = Vec::with_capacity(32);
    out.push(ARCP_PROTOCOL_VERSION);
    out.push(value.signal_type().wire_kind());
    out.extend_from_slice(&signal_id.to_le_bytes());
    encode_value_payload(value, &mut out)?;
    Ok(out)
}

pub fn decode_update(frame: &[u8]) -> Result<(u16, SignalValue), &'static str> {
    if frame.len() < 4 {
        return Err("frame too small");
    }
    if frame[0] != ARCP_PROTOCOL_VERSION {
        return Err("unsupported protocol version");
    }
    let signal_type = SignalType::try_from(frame[1])?;
    let signal_id = u16::from_le_bytes([frame[2], frame[3]]);
    let value = decode_value_payload(signal_type, &frame[4..])?;
    Ok((signal_id, value))
}

pub fn encode_event(event: &RuntimeEvent) -> Result<Vec<u8>, &'static str> {
    let mut out = Vec::with_capacity(32);
    out.push(ARCP_PROTOCOL_VERSION);
    match event {
        RuntimeEvent::TunableSet { signal_id, value } => {
            out.push(EVENT_TUNABLE_SET);
            out.extend_from_slice(&signal_id.to_le_bytes());
            out.push(value.signal_type().wire_kind());
            encode_value_payload(value, &mut out)?;
        }
        RuntimeEvent::Action { signal_id } => {
            out.push(EVENT_ACTION);
            out.extend_from_slice(&signal_id.to_le_bytes());
            out.push(0);
        }
    }
    Ok(out)
}

pub fn decode_event(data: &[u8]) -> Result<RuntimeEvent, &'static str> {
    if data.len() < 5 {
        return Err("event too small");
    }
    if data[0] != ARCP_PROTOCOL_VERSION {
        return Err("unsupported protocol version");
    }
    let event_kind = data[1];
    let signal_id = u16::from_le_bytes([data[2], data[3]]);
    match event_kind {
        EVENT_TUNABLE_SET => {
            let signal_type = SignalType::try_from(data[4])?;
            let value = decode_value_payload(signal_type, &data[5..])?;
            Ok(RuntimeEvent::TunableSet { signal_id, value })
        }
        EVENT_ACTION => Ok(RuntimeEvent::Action { signal_id }),
        _ => Err("unsupported event kind"),
    }
}

fn encode_value_payload(value: &SignalValue, out: &mut Vec<u8>) -> Result<(), &'static str> {
    match value {
        SignalValue::Bool(v) => out.push(if *v { 1 } else { 0 }),
        SignalValue::I64(v) => out.extend_from_slice(&v.to_le_bytes()),
        SignalValue::F64(v) => out.extend_from_slice(&v.to_le_bytes()),
        SignalValue::Str(v) => encode_string(v, out)?,
        SignalValue::BoolArray(values) => {
            encode_len(values.len(), out)?;
            out.extend(values.iter().map(|v| if *v { 1 } else { 0 }));
        }
        SignalValue::I64Array(values) => {
            encode_len(values.len(), out)?;
            for value in values {
                out.extend_from_slice(&value.to_le_bytes());
            }
        }
        SignalValue::F64Array(values) => {
            encode_len(values.len(), out)?;
            for value in values {
                out.extend_from_slice(&value.to_le_bytes());
            }
        }
        SignalValue::StrArray(values) => {
            encode_len(values.len(), out)?;
            for value in values {
                encode_string(value, out)?;
            }
        }
    }
    Ok(())
}

fn decode_value_payload(
    signal_type: SignalType,
    payload: &[u8],
) -> Result<SignalValue, &'static str> {
    match signal_type {
        SignalType::Bool => {
            if payload.len() != 1 {
                return Err("invalid bool payload size");
            }
            Ok(SignalValue::Bool(payload[0] != 0))
        }
        SignalType::I64 => {
            if payload.len() != 8 {
                return Err("invalid i64 payload size");
            }
            let mut raw = [0_u8; 8];
            raw.copy_from_slice(payload);
            Ok(SignalValue::I64(i64::from_le_bytes(raw)))
        }
        SignalType::F64 => {
            if payload.len() != 8 {
                return Err("invalid f64 payload size");
            }
            let mut raw = [0_u8; 8];
            raw.copy_from_slice(payload);
            Ok(SignalValue::F64(f64::from_le_bytes(raw)))
        }
        SignalType::Str => {
            let (value, consumed) = decode_string(payload)?;
            if consumed != payload.len() {
                return Err("invalid string payload size");
            }
            Ok(SignalValue::Str(value))
        }
        SignalType::BoolArray => {
            let (count, mut offset) = decode_len(payload)?;
            if payload.len().saturating_sub(offset) != count {
                return Err("invalid bool[] payload size");
            }
            let mut values = Vec::with_capacity(count);
            for byte in &payload[offset..] {
                values.push(*byte != 0);
            }
            offset += count;
            if offset != payload.len() {
                return Err("invalid bool[] payload");
            }
            Ok(SignalValue::BoolArray(values))
        }
        SignalType::I64Array => {
            let (count, offset) = decode_len(payload)?;
            let expected = count.checked_mul(8).ok_or("i64[] length overflow")?;
            if payload.len().saturating_sub(offset) != expected {
                return Err("invalid i64[] payload size");
            }
            let mut values = Vec::with_capacity(count);
            let mut index = offset;
            while index < payload.len() {
                let mut raw = [0_u8; 8];
                raw.copy_from_slice(&payload[index..index + 8]);
                values.push(i64::from_le_bytes(raw));
                index += 8;
            }
            Ok(SignalValue::I64Array(values))
        }
        SignalType::F64Array => Ok(SignalValue::F64Array(decode_f64_array_payload(payload)?)),
        SignalType::StrArray => {
            let (count, mut offset) = decode_len(payload)?;
            let mut values = Vec::with_capacity(count);
            for _ in 0..count {
                let (value, consumed) = decode_string(&payload[offset..])?;
                values.push(value);
                offset += consumed;
            }
            if offset != payload.len() {
                return Err("invalid string[] payload size");
            }
            Ok(SignalValue::StrArray(values))
        }
    }
}

fn decode_f64_array_payload(payload: &[u8]) -> Result<Vec<f64>, &'static str> {
    let (count, offset) = decode_len(payload)?;
    let expected = count.checked_mul(8).ok_or("f64[] length overflow")?;
    if payload.len().saturating_sub(offset) != expected {
        return Err("invalid f64[] payload size");
    }

    let bytes = &payload[offset..];

    #[cfg(all(target_arch = "aarch64", target_endian = "little"))]
    {
        return Ok(decode_f64_array_neon(bytes, count));
    }

    #[allow(unreachable_code)]
    Ok(decode_f64_array_scalar(bytes, count))
}

fn decode_f64_array_scalar(bytes: &[u8], count: usize) -> Vec<f64> {
    let mut values = Vec::with_capacity(count);
    let mut index = 0_usize;
    while index < bytes.len() {
        let mut raw = [0_u8; 8];
        raw.copy_from_slice(&bytes[index..index + 8]);
        values.push(f64::from_le_bytes(raw));
        index += 8;
    }
    values
}

#[cfg(all(target_arch = "aarch64", target_endian = "little"))]
fn decode_f64_array_neon(bytes: &[u8], count: usize) -> Vec<f64> {
    use core::arch::aarch64::{vld1q_u8, vst1q_u8};

    let mut values = vec![0_f64; count];
    if bytes.is_empty() {
        return values;
    }

    // On little-endian AArch64, ARCP f64 payload bytes can be copied directly.
    unsafe {
        let src = bytes.as_ptr();
        let dst = values.as_mut_ptr() as *mut u8;
        let mut offset = 0_usize;

        while offset + 16 <= bytes.len() {
            let chunk = vld1q_u8(src.add(offset));
            vst1q_u8(dst.add(offset), chunk);
            offset += 16;
        }
        if offset < bytes.len() {
            std::ptr::copy_nonoverlapping(src.add(offset), dst.add(offset), bytes.len() - offset);
        }
    }

    values
}

fn encode_len(len: usize, out: &mut Vec<u8>) -> Result<(), &'static str> {
    let len_u16 = u16::try_from(len).map_err(|_| "length exceeds u16 max")?;
    out.extend_from_slice(&len_u16.to_le_bytes());
    Ok(())
}

fn decode_len(data: &[u8]) -> Result<(usize, usize), &'static str> {
    if data.len() < 2 {
        return Err("length prefix missing");
    }
    let len = u16::from_le_bytes([data[0], data[1]]) as usize;
    Ok((len, 2))
}

fn encode_string(value: &str, out: &mut Vec<u8>) -> Result<(), &'static str> {
    encode_len(value.len(), out)?;
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn decode_string(data: &[u8]) -> Result<(String, usize), &'static str> {
    let (len, offset) = decode_len(data)?;
    if data.len() < offset + len {
        return Err("string payload truncated");
    }
    let bytes = &data[offset..offset + len];
    let value = std::str::from_utf8(bytes).map_err(|_| "string payload is not utf8")?;
    Ok((value.to_string(), offset + len))
}

#[cfg(test)]
mod tests {
    use super::{decode_event, decode_update, encode_event, encode_update};
    use crate::{RuntimeEvent, SignalValue};

    #[test]
    fn round_trip_scalar_update() {
        let frame = encode_update(42, &SignalValue::F64(12.5)).expect("encode");
        let (signal_id, value) = decode_update(&frame).expect("decode");
        assert_eq!(signal_id, 42);
        assert_eq!(value, SignalValue::F64(12.5));
    }

    #[test]
    fn round_trip_string_array_update() {
        let frame = encode_update(
            10,
            &SignalValue::StrArray(vec!["alpha".to_string(), "beta".to_string()]),
        )
        .expect("encode");
        let (signal_id, value) = decode_update(&frame).expect("decode");
        assert_eq!(signal_id, 10);
        assert_eq!(
            value,
            SignalValue::StrArray(vec!["alpha".to_string(), "beta".to_string()])
        );
    }

    #[test]
    fn round_trip_f64_array_update() {
        let frame =
            encode_update(11, &SignalValue::F64Array(vec![1.0, -2.5, 3.125])).expect("encode");
        let (signal_id, value) = decode_update(&frame).expect("decode");
        assert_eq!(signal_id, 11);
        assert_eq!(value, SignalValue::F64Array(vec![1.0, -2.5, 3.125]));
    }

    #[test]
    fn round_trip_tunable_event() {
        let encoded = encode_event(&RuntimeEvent::TunableSet {
            signal_id: 7,
            value: SignalValue::I64(99),
        })
        .expect("encode");
        let decoded = decode_event(&encoded).expect("decode");
        assert_eq!(
            decoded,
            RuntimeEvent::TunableSet {
                signal_id: 7,
                value: SignalValue::I64(99),
            }
        );
    }

    #[test]
    fn round_trip_action_event() {
        let encoded = encode_event(&RuntimeEvent::Action { signal_id: 5 }).expect("encode");
        let decoded = decode_event(&encoded).expect("decode");
        assert_eq!(decoded, RuntimeEvent::Action { signal_id: 5 });
    }
}
