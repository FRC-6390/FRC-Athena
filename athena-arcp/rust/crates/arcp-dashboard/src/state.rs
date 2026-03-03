use std::collections::BTreeMap;
use std::time::Instant;

use arcp_core::SignalValue;

use crate::manifest::ManifestItem;

pub struct DashboardState {
    descriptors: Vec<ManifestItem>,
    latest_values: BTreeMap<u16, SignalValue>,
    last_update_by_signal: BTreeMap<u16, u64>,
    update_count: u64,
    revision: u64,
    manifest_revision: u64,
    started_at: Instant,
}

impl DashboardState {
    pub fn new(mut descriptors: Vec<ManifestItem>) -> Self {
        descriptors.sort_by_key(|item| item.signal_id);
        Self {
            descriptors,
            latest_values: BTreeMap::new(),
            last_update_by_signal: BTreeMap::new(),
            update_count: 0,
            revision: 0,
            manifest_revision: 0,
            started_at: Instant::now(),
        }
    }

    pub fn apply_update(&mut self, signal_id: u16, value: SignalValue) {
        self.update_count = self.update_count.saturating_add(1);
        self.latest_values.insert(signal_id, value);
        self.last_update_by_signal
            .insert(signal_id, self.update_count);
        self.revision = self.revision.saturating_add(1);
    }

    pub fn replace_descriptors(&mut self, mut descriptors: Vec<ManifestItem>) {
        descriptors.sort_by_key(|item| item.signal_id);
        self.descriptors = descriptors;
        self.latest_values.clear();
        self.last_update_by_signal.clear();
        self.revision = self.revision.saturating_add(1);
        self.manifest_revision = self.manifest_revision.saturating_add(1);
    }

    pub fn upsert_descriptor(&mut self, descriptor: ManifestItem) {
        match self
            .descriptors
            .binary_search_by_key(&descriptor.signal_id, |item| item.signal_id)
        {
            Ok(index) => {
                self.descriptors[index] = descriptor;
            }
            Err(index) => {
                self.descriptors.insert(index, descriptor);
            }
        }
        self.revision = self.revision.saturating_add(1);
        self.manifest_revision = self.manifest_revision.saturating_add(1);
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

    pub fn manifest_revision(&self) -> u64 {
        self.manifest_revision
    }

    pub fn updated_signal_ids_since(&self, since_update_count: u64) -> Vec<u16> {
        self.last_update_by_signal
            .iter()
            .filter_map(|(signal_id, last_update)| {
                if *last_update > since_update_count {
                    Some(*signal_id)
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn descriptor_for(&self, signal_id: u16) -> Option<&ManifestItem> {
        self.descriptors
            .binary_search_by_key(&signal_id, |descriptor| descriptor.signal_id)
            .ok()
            .and_then(|index| self.descriptors.get(index))
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn uptime_ms(&self) -> u64 {
        self.started_at.elapsed().as_millis() as u64
    }
}
