use arcp_core::{SignalType, SignalValue};

pub(crate) fn parse_value_for_type(
    signal_type: SignalType,
    raw: &str,
) -> Result<SignalValue, &'static str> {
    let input = raw.trim();
    match signal_type {
        SignalType::Bool => parse_bool(input).map(SignalValue::Bool),
        SignalType::I64 => input
            .parse::<i64>()
            .map(SignalValue::I64)
            .map_err(|_| "invalid i64"),
        SignalType::F64 => input
            .parse::<f64>()
            .map(SignalValue::F64)
            .map_err(|_| "invalid f64"),
        SignalType::Str => parse_string(input).map(SignalValue::Str),
        SignalType::BoolArray => parse_bool_array(input).map(SignalValue::BoolArray),
        SignalType::I64Array => parse_i64_array(input).map(SignalValue::I64Array),
        SignalType::F64Array => parse_f64_array(input).map(SignalValue::F64Array),
        SignalType::StrArray => parse_string_array(input).map(SignalValue::StrArray),
    }
}

fn parse_bool(input: &str) -> Result<bool, &'static str> {
    if input.eq_ignore_ascii_case("true") || input == "1" {
        return Ok(true);
    }
    if input.eq_ignore_ascii_case("false") || input == "0" {
        return Ok(false);
    }
    Err("invalid bool")
}

fn parse_string(input: &str) -> Result<String, &'static str> {
    if input.starts_with('"') && input.ends_with('"') && input.len() >= 2 {
        return Ok(input[1..input.len() - 1].to_string());
    }
    Ok(input.to_string())
}

fn parse_bool_array(input: &str) -> Result<Vec<bool>, &'static str> {
    parse_list(input, parse_bool)
}

fn parse_i64_array(input: &str) -> Result<Vec<i64>, &'static str> {
    parse_list(input, |part| part.parse::<i64>().map_err(|_| "invalid i64"))
}

fn parse_f64_array(input: &str) -> Result<Vec<f64>, &'static str> {
    parse_list(input, |part| part.parse::<f64>().map_err(|_| "invalid f64"))
}

fn parse_string_array(input: &str) -> Result<Vec<String>, &'static str> {
    parse_list(input, parse_string)
}

fn parse_list<T, F>(input: &str, parse_item: F) -> Result<Vec<T>, &'static str>
where
    F: Fn(&str) -> Result<T, &'static str>,
{
    let trimmed = input.trim();
    if !trimmed.starts_with('[') || !trimmed.ends_with(']') {
        return Err("array format must be [..]");
    }
    let inner = &trimmed[1..trimmed.len() - 1];
    if inner.trim().is_empty() {
        return Ok(Vec::new());
    }
    let mut out = Vec::new();
    for token in inner.split(',') {
        out.push(parse_item(token.trim())?);
    }
    Ok(out)
}
