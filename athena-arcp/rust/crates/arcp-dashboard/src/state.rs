use std::collections::BTreeMap;
use std::time::Instant;

use arcp_core::SignalValue;

use crate::manifest::ManifestItem;

pub struct DashboardState {
    descriptors: Vec<ManifestItem>,
    latest_values: BTreeMap<u16, SignalValue>,
    update_count: u64,
    revision: u64,
    started_at: Instant,
}

impl DashboardState {
    pub fn new(mut descriptors: Vec<ManifestItem>) -> Self {
        descriptors.sort_by_key(|item| item.signal_id);
        Self {
            descriptors,
            latest_values: BTreeMap::new(),
            update_count: 0,
            revision: 0,
            started_at: Instant::now(),
        }
    }

    pub fn apply_update(&mut self, signal_id: u16, value: SignalValue) {
        self.latest_values.insert(signal_id, value);
        self.update_count = self.update_count.saturating_add(1);
        self.revision = self.revision.saturating_add(1);
    }

    pub fn replace_descriptors(&mut self, mut descriptors: Vec<ManifestItem>) {
        descriptors.sort_by_key(|item| item.signal_id);
        self.descriptors = descriptors;
        self.latest_values.clear();
        self.revision = self.revision.saturating_add(1);
    }

    pub fn descriptors(&self) -> &[ManifestItem] {
        &self.descriptors
    }

    pub fn value_for(&self, signal_id: u16) -> Option<&SignalValue> {
        self.latest_values.get(&signal_id)
    }

    pub fn update_count(&self) -> u64 {
        self.update_count
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn uptime_ms(&self) -> u64 {
        self.started_at.elapsed().as_millis() as u64
    }
}
