use std::fmt::Write as _;

use arcp_core::SignalValue;

use crate::state::DashboardState;

pub fn render_state_json(state: &DashboardState) -> String {
    let mut out = String::with_capacity(256 + state.descriptors().len() * 128);
    let _ = write!(
        out,
        "{{\"uptime_ms\":{},\"update_count\":{},\"signal_count\":{},\"signals\":[",
        state.uptime_ms(),
        state.update_count(),
        state.descriptors().len()
    );

    for (index, descriptor) in state.descriptors().iter().enumerate() {
        if index > 0 {
            out.push(',');
        }
        out.push('{');
        let _ = write!(out, "\"id\":{}", descriptor.signal_id);

        out.push_str(",\"path\":\"");
        push_json_escaped(&mut out, &descriptor.path);
        out.push('"');

        out.push_str(",\"type\":\"");
        out.push_str(descriptor.signal_type.as_str());
        out.push('"');

        out.push_str(",\"kind\":\"");
        out.push_str(descriptor.kind.as_str());
        out.push('"');

        out.push_str(",\"access\":\"");
        out.push_str(descriptor.access.as_str());
        out.push('"');

        out.push_str(",\"policy\":\"");
        out.push_str(descriptor.policy.as_str());
        out.push('"');

        out.push_str(",\"durability\":\"");
        out.push_str(descriptor.durability.as_str());
        out.push('"');

        out.push_str(",\"value\":");
        if let Some(value) = state.value_for(descriptor.signal_id) {
            push_value_json(&mut out, value);
        } else {
            out.push_str("null");
        }
        out.push('}');
    }

    out.push_str("]}");
    out
}

fn push_value_json(out: &mut String, value: &SignalValue) {
    match value {
        SignalValue::Bool(v) => out.push_str(if *v { "true" } else { "false" }),
        SignalValue::I64(v) => {
            let _ = write!(out, "{v}");
        }
        SignalValue::F64(v) => {
            if v.is_finite() {
                let _ = write!(out, "{v}");
            } else {
                out.push_str("null");
            }
        }
        SignalValue::Str(v) => {
            out.push('"');
            push_json_escaped(out, v);
            out.push('"');
        }
        SignalValue::BoolArray(values) => {
            out.push('[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 {
                    out.push(',');
                }
                out.push_str(if *value { "true" } else { "false" });
            }
            out.push(']');
        }
        SignalValue::I64Array(values) => {
            out.push('[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 {
                    out.push(',');
                }
                let _ = write!(out, "{value}");
            }
            out.push(']');
        }
        SignalValue::F64Array(values) => {
            out.push('[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 {
                    out.push(',');
                }
                if value.is_finite() {
                    let _ = write!(out, "{value}");
                } else {
                    out.push_str("null");
                }
            }
            out.push(']');
        }
        SignalValue::StrArray(values) => {
            out.push('[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 {
                    out.push(',');
                }
                out.push('"');
                push_json_escaped(out, value);
                out.push('"');
            }
            out.push(']');
        }
    }
}

fn push_json_escaped(out: &mut String, raw: &str) {
    for ch in raw.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c.is_control() => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
}
