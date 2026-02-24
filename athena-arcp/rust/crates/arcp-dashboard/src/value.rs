use std::fmt::Write as _;

use arcp_core::SignalValue;

const ARRAY_PREVIEW_LIMIT: usize = 8;

pub fn format_signal_value(value: &SignalValue) -> String {
    match value {
        SignalValue::Bool(v) => v.to_string(),
        SignalValue::I64(v) => v.to_string(),
        SignalValue::F64(v) => format!("{v:.6}"),
        SignalValue::Str(v) => v.clone(),
        SignalValue::BoolArray(values) => format_bool_array(values),
        SignalValue::I64Array(values) => format_numeric_array(values),
        SignalValue::F64Array(values) => format_float_array(values),
        SignalValue::StrArray(values) => format_string_array(values),
    }
}

fn format_bool_array(values: &[bool]) -> String {
    let mut out = String::new();
    out.push('[');
    for (index, value) in values.iter().take(ARRAY_PREVIEW_LIMIT).enumerate() {
        if index > 0 {
            out.push_str(", ");
        }
        out.push_str(if *value { "true" } else { "false" });
    }
    append_array_trailer(&mut out, values.len());
    out
}

fn format_numeric_array<T>(values: &[T]) -> String
where
    T: std::fmt::Display,
{
    let mut out = String::new();
    out.push('[');
    for (index, value) in values.iter().take(ARRAY_PREVIEW_LIMIT).enumerate() {
        if index > 0 {
            out.push_str(", ");
        }
        let _ = write!(out, "{value}");
    }
    append_array_trailer(&mut out, values.len());
    out
}

fn format_float_array(values: &[f64]) -> String {
    let mut out = String::new();
    out.push('[');
    for (index, value) in values.iter().take(ARRAY_PREVIEW_LIMIT).enumerate() {
        if index > 0 {
            out.push_str(", ");
        }
        let _ = write!(out, "{value:.6}");
    }
    append_array_trailer(&mut out, values.len());
    out
}

fn format_string_array(values: &[String]) -> String {
    let mut out = String::new();
    out.push('[');
    for (index, value) in values.iter().take(ARRAY_PREVIEW_LIMIT).enumerate() {
        if index > 0 {
            out.push_str(", ");
        }
        out.push('"');
        out.push_str(value);
        out.push('"');
    }
    append_array_trailer(&mut out, values.len());
    out
}

fn append_array_trailer(out: &mut String, len: usize) {
    if len > ARRAY_PREVIEW_LIMIT {
        let _ = write!(out, ", ... ({len})");
    }
    out.push(']');
}
