use std::convert::TryFrom;

use crate::constants::{
    FRAME_BOOL, FRAME_BOOL_ARRAY, FRAME_F64, FRAME_F64_ARRAY, FRAME_I64, FRAME_I64_ARRAY,
    FRAME_STR, FRAME_STR_ARRAY,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SignalType {
    Bool = FRAME_BOOL,
    I64 = FRAME_I64,
    F64 = FRAME_F64,
    Str = FRAME_STR,
    BoolArray = FRAME_BOOL_ARRAY,
    I64Array = FRAME_I64_ARRAY,
    F64Array = FRAME_F64_ARRAY,
    StrArray = FRAME_STR_ARRAY,
}

impl SignalType {
    pub fn wire_kind(self) -> u8 {
        self as u8
    }

    pub fn as_str(self) -> &'static str {
        match self {
            SignalType::Bool => "bool",
            SignalType::I64 => "i64",
            SignalType::F64 => "f64",
            SignalType::Str => "string",
            SignalType::BoolArray => "bool[]",
            SignalType::I64Array => "i64[]",
            SignalType::F64Array => "f64[]",
            SignalType::StrArray => "string[]",
        }
    }
}

impl TryFrom<u8> for SignalType {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            FRAME_BOOL => Ok(SignalType::Bool),
            FRAME_I64 => Ok(SignalType::I64),
            FRAME_F64 => Ok(SignalType::F64),
            FRAME_STR => Ok(SignalType::Str),
            FRAME_BOOL_ARRAY => Ok(SignalType::BoolArray),
            FRAME_I64_ARRAY => Ok(SignalType::I64Array),
            FRAME_F64_ARRAY => Ok(SignalType::F64Array),
            FRAME_STR_ARRAY => Ok(SignalType::StrArray),
            _ => Err("unsupported signal type"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SignalKind {
    Telemetry = 1,
    Command = 2,
}

impl SignalKind {
    pub fn as_str(self) -> &'static str {
        match self {
            SignalKind::Telemetry => "telemetry",
            SignalKind::Command => "command",
        }
    }
}

impl TryFrom<u8> for SignalKind {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(SignalKind::Telemetry),
            2 => Ok(SignalKind::Command),
            _ => Err("unsupported signal kind"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SignalAccess {
    Observe = 1,
    Write = 2,
    Invoke = 3,
}

impl SignalAccess {
    pub fn as_str(self) -> &'static str {
        match self {
            SignalAccess::Observe => "observe",
            SignalAccess::Write => "write",
            SignalAccess::Invoke => "invoke",
        }
    }
}

impl TryFrom<u8> for SignalAccess {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(SignalAccess::Observe),
            2 => Ok(SignalAccess::Write),
            3 => Ok(SignalAccess::Invoke),
            _ => Err("unsupported signal access"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SignalPolicy {
    HighRate = 1,
    OnChange = 2,
    Sampled = 3,
    SnapshotOnly = 4,
}

impl SignalPolicy {
    pub fn as_str(self) -> &'static str {
        match self {
            SignalPolicy::HighRate => "high_rate",
            SignalPolicy::OnChange => "on_change",
            SignalPolicy::Sampled => "sampled",
            SignalPolicy::SnapshotOnly => "snapshot_only",
        }
    }
}

impl TryFrom<u8> for SignalPolicy {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(SignalPolicy::HighRate),
            2 => Ok(SignalPolicy::OnChange),
            3 => Ok(SignalPolicy::Sampled),
            4 => Ok(SignalPolicy::SnapshotOnly),
            _ => Err("unsupported signal policy"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SignalDurability {
    Volatile = 1,
    Retained = 2,
    Persistent = 3,
}

impl SignalDurability {
    pub fn as_str(self) -> &'static str {
        match self {
            SignalDurability::Volatile => "volatile",
            SignalDurability::Retained => "retained",
            SignalDurability::Persistent => "persistent",
        }
    }
}

impl TryFrom<u8> for SignalDurability {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(SignalDurability::Volatile),
            2 => Ok(SignalDurability::Retained),
            3 => Ok(SignalDurability::Persistent),
            _ => Err("unsupported signal durability"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PersistenceScope {
    None = 0,
    RobotFlash = 1,
    HostWorkspace = 2,
}

impl PersistenceScope {
    pub fn as_str(self) -> &'static str {
        match self {
            PersistenceScope::None => "none",
            PersistenceScope::RobotFlash => "robot_flash",
            PersistenceScope::HostWorkspace => "host_workspace",
        }
    }
}

impl TryFrom<u8> for PersistenceScope {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(PersistenceScope::None),
            1 => Ok(PersistenceScope::RobotFlash),
            2 => Ok(PersistenceScope::HostWorkspace),
            _ => Err("unsupported persistence scope"),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum SignalValue {
    Bool(bool),
    I64(i64),
    F64(f64),
    Str(String),
    BoolArray(Vec<bool>),
    I64Array(Vec<i64>),
    F64Array(Vec<f64>),
    StrArray(Vec<String>),
}

impl SignalValue {
    pub fn signal_type(&self) -> SignalType {
        match self {
            SignalValue::Bool(_) => SignalType::Bool,
            SignalValue::I64(_) => SignalType::I64,
            SignalValue::F64(_) => SignalType::F64,
            SignalValue::Str(_) => SignalType::Str,
            SignalValue::BoolArray(_) => SignalType::BoolArray,
            SignalValue::I64Array(_) => SignalType::I64Array,
            SignalValue::F64Array(_) => SignalType::F64Array,
            SignalValue::StrArray(_) => SignalType::StrArray,
        }
    }
}

#[derive(Debug, Clone)]
pub struct SignalDescriptor {
    pub signal_id: u16,
    pub signal_type: SignalType,
    pub kind: SignalKind,
    pub access: SignalAccess,
    pub policy: SignalPolicy,
    pub durability: SignalDurability,
    pub persistence_scope: PersistenceScope,
    pub persistence_key: Option<String>,
    pub metadata_version: u32,
    pub metadata_hash: u64,
    pub path: String,
}

impl SignalDescriptor {
    pub fn new(
        signal_id: u16,
        signal_type: SignalType,
        kind: SignalKind,
        access: SignalAccess,
        policy: SignalPolicy,
        durability: SignalDurability,
        path: impl Into<String>,
    ) -> Self {
        let persistence_scope = match durability {
            SignalDurability::Persistent => PersistenceScope::RobotFlash,
            _ => PersistenceScope::None,
        };
        Self {
            signal_id,
            signal_type,
            kind,
            access,
            policy,
            durability,
            persistence_scope,
            persistence_key: None,
            metadata_version: 1,
            metadata_hash: 0,
            path: path.into(),
        }
    }

    pub fn command(signal_id: u16, signal_type: SignalType, path: impl Into<String>) -> Self {
        Self::new(
            signal_id,
            signal_type,
            SignalKind::Command,
            SignalAccess::Invoke,
            SignalPolicy::SnapshotOnly,
            SignalDurability::Volatile,
            path,
        )
    }

    pub fn telemetry(
        signal_id: u16,
        signal_type: SignalType,
        access: SignalAccess,
        policy: SignalPolicy,
        durability: SignalDurability,
        path: impl Into<String>,
    ) -> Self {
        Self::new(
            signal_id,
            signal_type,
            SignalKind::Telemetry,
            access,
            policy,
            durability,
            path,
        )
    }

    pub fn apply_metadata_hash(&mut self, metadata_hash: u64) {
        self.metadata_hash = metadata_hash;
        self.metadata_version = self.metadata_version.saturating_add(1);
    }

    pub fn metadata_fingerprint(&self) -> u64 {
        let mut hasher = Fnv64::new();
        hasher.push_u16(self.signal_id);
        hasher.push_u8(self.signal_type as u8);
        hasher.push_u8(self.kind as u8);
        hasher.push_u8(self.access as u8);
        hasher.push_u8(self.policy as u8);
        hasher.push_u8(self.durability as u8);
        hasher.push_u8(self.persistence_scope as u8);
        hasher.push_u32(self.metadata_version);
        hasher.push_str(&self.path);
        if let Some(key) = &self.persistence_key {
            hasher.push_str(key);
        }
        hasher.finish()
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        if self.path.is_empty() {
            return Err("path must not be empty");
        }
        if self.path.len() > 255 {
            return Err("path exceeds max length");
        }
        if self.durability != SignalDurability::Persistent
            && self.persistence_scope != PersistenceScope::None
        {
            return Err("persistence scope requires persistent durability");
        }
        if self.persistence_scope == PersistenceScope::None && self.persistence_key.is_some() {
            return Err("persistence key set without persistence scope");
        }
        if self.kind == SignalKind::Command {
            if self.access != SignalAccess::Invoke {
                return Err("command kind requires invoke access");
            }
            if self.policy != SignalPolicy::SnapshotOnly {
                return Err("command kind requires snapshot_only policy");
            }
        }
        if self.access == SignalAccess::Invoke && self.kind != SignalKind::Command {
            return Err("invoke access requires command kind");
        }
        Ok(())
    }
}

struct Fnv64 {
    state: u64,
}

impl Fnv64 {
    const OFFSET: u64 = 0xcbf29ce484222325;
    const PRIME: u64 = 0x00000100000001B3;

    fn new() -> Self {
        Self {
            state: Self::OFFSET,
        }
    }

    fn push_u8(&mut self, value: u8) {
        self.state ^= value as u64;
        self.state = self.state.wrapping_mul(Self::PRIME);
    }

    fn push_u16(&mut self, value: u16) {
        for byte in value.to_le_bytes() {
            self.push_u8(byte);
        }
    }

    fn push_u32(&mut self, value: u32) {
        for byte in value.to_le_bytes() {
            self.push_u8(byte);
        }
    }

    fn push_str(&mut self, value: &str) {
        self.push_u16(value.len().min(u16::MAX as usize) as u16);
        for byte in value.as_bytes() {
            self.push_u8(*byte);
        }
    }

    fn finish(self) -> u64 {
        self.state
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum RuntimeEvent {
    TunableSet { signal_id: u16, value: SignalValue },
    Action { signal_id: u16 },
}
