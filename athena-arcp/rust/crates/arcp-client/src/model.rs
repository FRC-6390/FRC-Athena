//! Dictionary, struct schemas and the latest value of every signal, as last seen by the client.

use std::collections::HashMap;

use arcp_core::v2::record::{DictEntry, Dictionary, StructSchema};
use arcp_core::v2::Value;

/// Role of lock-state signals (appendix E1-W, "Lock state on the wire").
const ROLE_LOCK: u8 = 7;
const LOCK_PREFIX: &str = "athena/locks/";
const STATUS_PREFIX: &str = "athena/arcp/status/";

/// The server's own status, read from the `athena/arcp/status/*` system signals.
#[derive(Debug, Clone, PartialEq)]
pub struct ServerStatus {
    pub clients: i64,
    pub bytes_per_second: f64,
    pub budget_bytes_per_second: f64,
    /// True when the bandwidth budget has lowered at least one subscription rate.
    pub throttled: bool,
    /// Patterns of the throttled subscriptions.
    pub throttled_patterns: Vec<String>,
    pub dropped_frames: i64,
}

#[derive(Debug, Clone, Default)]
pub struct Model {
    dictionary: Dictionary,
    schemas: Vec<StructSchema>,
    index_by_id: HashMap<u64, usize>,
    id_by_path: HashMap<String, u64>,
    latest: HashMap<u64, Value>,
}

impl Model {
    /// Replaces the dictionary and schemas (a `Schema` message or a replay dictionary record).
    /// Latest values are kept, so the UI does not blank out on a re-announce.
    pub fn set_dictionary(&mut self, dictionary: Dictionary, schemas: Vec<StructSchema>) {
        self.index_by_id = dictionary.entries.iter().enumerate().map(|(i, e)| (e.signal_id, i)).collect();
        self.id_by_path = dictionary.entries.iter().map(|e| (e.path.clone(), e.signal_id)).collect();
        self.dictionary = dictionary;
        self.schemas = schemas;
    }

    pub fn upsert_schema(&mut self, schema: StructSchema) {
        match self.schemas.iter_mut().find(|s| s.id == schema.id) {
            Some(existing) => *existing = schema,
            None => self.schemas.push(schema),
        }
    }

    pub fn dictionary(&self) -> &Dictionary {
        &self.dictionary
    }

    pub fn schemas(&self) -> &[StructSchema] {
        &self.schemas
    }

    pub fn entry(&self, signal_id: u64) -> Option<&DictEntry> {
        self.index_by_id.get(&signal_id).map(|&i| &self.dictionary.entries[i])
    }

    pub fn signal_id(&self, path: &str) -> Option<u64> {
        self.id_by_path.get(path).copied()
    }

    pub fn set_value(&mut self, signal_id: u64, value: Value) {
        self.latest.insert(signal_id, value);
    }

    pub fn value(&self, signal_id: u64) -> Option<&Value> {
        self.latest.get(&signal_id)
    }

    /// Latest server status, or `None` until the robot's first status frame arrives.
    pub fn server_status(&self) -> Option<ServerStatus> {
        let get = |name: &str| {
            let path = format!("{STATUS_PREFIX}{name}");
            let entry = self.dictionary.entries.iter().find(|e| e.path == path)?;
            self.latest.get(&entry.signal_id)
        };
        let as_i64 = |v: Option<&Value>| match v? {
            Value::I64(x) => Some(*x),
            _ => None,
        };
        let as_f64 = |v: Option<&Value>| match v? {
            Value::F64(x) => Some(*x),
            _ => None,
        };
        let patterns = match get("throttledPatterns")? {
            Value::StrArray(p) => p.clone(),
            _ => return None,
        };
        Some(ServerStatus {
            clients: as_i64(get("clients"))?,
            bytes_per_second: as_f64(get("bytesPerSecond"))?,
            budget_bytes_per_second: as_f64(get("budgetBytesPerSecond"))?,
            throttled: matches!(get("throttled")?, Value::Bool(true)),
            throttled_patterns: patterns,
            dropped_frames: as_i64(get("droppedFrames"))?,
        })
    }

    /// Current lock state of a lock group: `Some(true)` locked, `None` when the robot does not
    /// publish that group. A writable entry's `lockGroup` joins to the role-7 signal at
    /// `athena/locks/<group>`.
    pub fn lock_state(&self, group: &str) -> Option<bool> {
        let full = format!("{LOCK_PREFIX}{group}");
        let suffix = format!("/{group}");
        let lock = self
            .dictionary
            .entries
            .iter()
            .find(|e| e.role == ROLE_LOCK && (e.path == full || e.path.ends_with(&suffix)))?;
        match self.latest.get(&lock.signal_id)? {
            Value::Bool(locked) => Some(*locked),
            _ => None,
        }
    }
}
