use arcp_core::{
    PersistenceScope, SignalAccess, SignalDurability, SignalKind, SignalPolicy, SignalType,
};

#[derive(Debug, Clone)]
pub struct ManifestItem {
    pub signal_id: u16,
    pub signal_type: SignalType,
    pub kind: SignalKind,
    pub access: SignalAccess,
    pub policy: SignalPolicy,
    pub durability: SignalDurability,
    pub persistence_scope: PersistenceScope,
    pub metadata_version: u32,
    pub metadata_hash: u64,
    pub path: String,
}

impl ManifestItem {
    pub fn parse_line(line: &str) -> Result<Self, &'static str> {
        parse_item(line)
    }
}

fn parse_item(line: &str) -> Result<ManifestItem, &'static str> {
    let mut parts = line.splitn(11, ' ');
    let tag = parts.next().ok_or("manifest line missing tag")?;
    if tag != "ITEM" {
        return Err("manifest line does not start with ITEM");
    }
    let signal_id = parts
        .next()
        .ok_or("manifest line missing signal id")?
        .parse::<u16>()
        .map_err(|_| "manifest line has invalid signal id")?;
    let signal_type = parse_signal_type(parts.next().ok_or("manifest line missing type")?)?;
    let kind = parse_signal_kind(parts.next().ok_or("manifest line missing kind")?)?;
    let access = parse_signal_access(parts.next().ok_or("manifest line missing access")?)?;
    let policy = parse_signal_policy(parts.next().ok_or("manifest line missing policy")?)?;
    let durability =
        parse_signal_durability(parts.next().ok_or("manifest line missing durability")?)?;
    let metadata_version = parts
        .next()
        .ok_or("manifest line missing metadata version")?
        .parse::<u32>()
        .map_err(|_| "manifest line has invalid metadata version")?;
    let metadata_hash = parts
        .next()
        .ok_or("manifest line missing metadata hash")?
        .parse::<u64>()
        .map_err(|_| "manifest line has invalid metadata hash")?;
    let persistence_scope = parse_persistence_scope(
        parts
            .next()
            .ok_or("manifest line missing persistence scope")?,
    )?;
    let path = parts
        .next()
        .ok_or("manifest line missing path")?
        .to_string();

    Ok(ManifestItem {
        signal_id,
        signal_type,
        kind,
        access,
        policy,
        durability,
        persistence_scope,
        metadata_version,
        metadata_hash,
        path,
    })
}

fn parse_signal_type(raw: &str) -> Result<SignalType, &'static str> {
    match raw {
        "bool" => Ok(SignalType::Bool),
        "i64" => Ok(SignalType::I64),
        "f64" => Ok(SignalType::F64),
        "string" => Ok(SignalType::Str),
        "bool[]" => Ok(SignalType::BoolArray),
        "i64[]" => Ok(SignalType::I64Array),
        "f64[]" => Ok(SignalType::F64Array),
        "string[]" => Ok(SignalType::StrArray),
        _ => Err("unsupported signal type in manifest"),
    }
}

fn parse_signal_kind(raw: &str) -> Result<SignalKind, &'static str> {
    match raw {
        "telemetry" => Ok(SignalKind::Telemetry),
        "command" => Ok(SignalKind::Command),
        _ => Err("unsupported signal kind in manifest"),
    }
}

fn parse_signal_access(raw: &str) -> Result<SignalAccess, &'static str> {
    match raw {
        "observe" => Ok(SignalAccess::Observe),
        "write" => Ok(SignalAccess::Write),
        "invoke" => Ok(SignalAccess::Invoke),
        _ => Err("unsupported signal access in manifest"),
    }
}

fn parse_signal_policy(raw: &str) -> Result<SignalPolicy, &'static str> {
    match raw {
        "high_rate" => Ok(SignalPolicy::HighRate),
        "on_change" => Ok(SignalPolicy::OnChange),
        "sampled" => Ok(SignalPolicy::Sampled),
        "snapshot_only" => Ok(SignalPolicy::SnapshotOnly),
        _ => Err("unsupported signal policy in manifest"),
    }
}

fn parse_signal_durability(raw: &str) -> Result<SignalDurability, &'static str> {
    match raw {
        "volatile" => Ok(SignalDurability::Volatile),
        "retained" => Ok(SignalDurability::Retained),
        "persistent" => Ok(SignalDurability::Persistent),
        _ => Err("unsupported signal durability in manifest"),
    }
}

fn parse_persistence_scope(raw: &str) -> Result<PersistenceScope, &'static str> {
    match raw {
        "none" => Ok(PersistenceScope::None),
        "robot_flash" => Ok(PersistenceScope::RobotFlash),
        "host_workspace" => Ok(PersistenceScope::HostWorkspace),
        _ => Err("unsupported persistence scope in manifest"),
    }
}
