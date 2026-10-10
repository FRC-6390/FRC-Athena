//! Realtime frame reassembly (E1) and sequence gap detection.

use std::collections::BTreeMap;

use arcp_core::v2::record::{Frame, FRAME_FLAG_KEYFRAME};
use arcp_core::v2::Entry;

/// Incomplete sequences kept at once. Older ones are evicted and counted as dropped.
pub const MAX_PENDING_SEQUENCES: usize = 8;

/// A backwards jump up to this size is a late or reordered datagram. Anything larger means the
/// robot restarted and its sequence counter began again.
pub const REORDER_WINDOW: u32 = 64;

struct Pending {
    timestamp_micros: u64,
    keyframe: bool,
    parts: Vec<Option<Vec<Entry>>>,
    received: usize,
}

/// Joins the fragments of one cycle back into a whole frame.
#[derive(Default)]
pub struct Reassembler {
    pending: BTreeMap<u32, Pending>,
    dropped: u64,
}

impl Reassembler {
    /// Feeds one decoded frame record and returns every frame that is now complete.
    pub fn push(&mut self, frame: Frame) -> Vec<Frame> {
        if !frame.fragmented() || frame.fragment_count <= 1 {
            // A whole cycle. Incomplete older cycles can no longer finish in order.
            let seq = frame.sequence;
            let stale: Vec<u32> = self.pending.keys().copied().filter(|s| *s < seq).collect();
            for s in stale {
                self.pending.remove(&s);
                self.dropped += 1;
            }
            return vec![frame];
        }

        let count = usize::from(frame.fragment_count);
        let index = usize::from(frame.fragment_index);
        if index >= count {
            return Vec::new();
        }
        let seq = frame.sequence;
        let keyframe = frame.keyframe();
        let pending = self.pending.entry(seq).or_insert_with(|| Pending {
            timestamp_micros: frame.timestamp_micros,
            keyframe: false,
            parts: vec![None; count],
            received: 0,
        });
        if pending.parts.len() != count {
            // Fragments of the same cycle disagree about how many there are. Ignore this one.
            return Vec::new();
        }
        if pending.parts[index].is_none() {
            pending.parts[index] = Some(frame.entries);
            pending.received += 1;
            pending.keyframe |= keyframe;
        }
        let complete = pending.received == count;

        let mut out = Vec::new();
        if complete {
            if let Some(done) = self.pending.remove(&seq) {
                let entries: Vec<Entry> = done.parts.into_iter().flatten().flatten().collect();
                out.push(Frame {
                    flags: u8::from(done.keyframe) * FRAME_FLAG_KEYFRAME,
                    timestamp_micros: done.timestamp_micros,
                    sequence: seq,
                    fragment_index: 0,
                    fragment_count: 1,
                    entries,
                });
            }
        }
        while self.pending.len() > MAX_PENDING_SEQUENCES {
            let oldest = match self.pending.keys().next() {
                Some(&s) => s,
                None => break,
            };
            self.pending.remove(&oldest);
            self.dropped += 1;
        }
        out
    }

    /// Returns and resets the number of cycles abandoned as incomplete.
    pub fn take_dropped(&mut self) -> u64 {
        std::mem::take(&mut self.dropped)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeqEvent {
    /// The first frame seen on this connection.
    First,
    InOrder,
    /// A repeat, or a late datagram inside the reorder window. Ignore it.
    Duplicate,
    /// `lost` cycles from `from` to `to` (inclusive) never arrived.
    Gap { from: u32, to: u32, lost: u32 },
    /// The robot restarted its sequence counter.
    Restart,
}

/// Classifies each complete frame's sequence number against the previous one.
#[derive(Default)]
pub struct SequenceTracker {
    last: Option<u32>,
}

impl SequenceTracker {
    pub fn observe(&mut self, seq: u32) -> SeqEvent {
        let Some(last) = self.last else {
            self.last = Some(seq);
            return SeqEvent::First;
        };
        let delta = seq.wrapping_sub(last);
        if delta == 0 {
            SeqEvent::Duplicate
        } else if delta == 1 {
            self.last = Some(seq);
            SeqEvent::InOrder
        } else if delta < 0x8000_0000 {
            let from = last.wrapping_add(1);
            let to = seq.wrapping_sub(1);
            self.last = Some(seq);
            SeqEvent::Gap { from, to, lost: delta - 1 }
        } else if last.wrapping_sub(seq) <= REORDER_WINDOW {
            SeqEvent::Duplicate
        } else {
            self.last = Some(seq);
            SeqEvent::Restart
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arcp_core::v2::Value;

    fn part(seq: u32, index: u16, count: u16, signal: u64) -> Frame {
        Frame {
            flags: 0x02 | if index == 0 { 0x01 } else { 0 },
            timestamp_micros: 99,
            sequence: seq,
            fragment_index: index,
            fragment_count: count,
            entries: vec![Entry { signal_id: signal, value: Value::I64(i64::from(index)) }],
        }
    }

    #[test]
    fn fragments_join_in_any_order() {
        let mut r = Reassembler::default();
        assert!(r.push(part(5, 2, 3, 30)).is_empty());
        assert!(r.push(part(5, 0, 3, 10)).is_empty());
        let out = r.push(part(5, 1, 3, 20));
        assert_eq!(out.len(), 1);
        let frame = &out[0];
        assert_eq!(frame.sequence, 5);
        assert!(frame.keyframe());
        assert!(!frame.fragmented());
        let signals: Vec<u64> = frame.entries.iter().map(|e| e.signal_id).collect();
        assert_eq!(signals, vec![10, 20, 30]);
        assert_eq!(r.take_dropped(), 0);
    }

    #[test]
    fn incomplete_cycle_is_evicted_when_a_newer_one_completes() {
        let mut r = Reassembler::default();
        assert!(r.push(part(7, 0, 2, 1)).is_empty());
        let whole = Frame::new(false, 1, 8, vec![]);
        assert_eq!(r.push(whole).len(), 1);
        assert_eq!(r.take_dropped(), 1);
    }

    #[test]
    fn sequence_tracker_reports_gaps_duplicates_and_restarts() {
        let mut t = SequenceTracker::default();
        assert_eq!(t.observe(10), SeqEvent::First);
        assert_eq!(t.observe(11), SeqEvent::InOrder);
        assert_eq!(t.observe(14), SeqEvent::Gap { from: 12, to: 13, lost: 2 });
        assert_eq!(t.observe(14), SeqEvent::Duplicate);
        assert_eq!(t.observe(13), SeqEvent::Duplicate);
        assert_eq!(t.observe(15), SeqEvent::InOrder);
        assert_eq!(t.observe(2000), SeqEvent::Gap { from: 16, to: 1999, lost: 1984 });
        assert_eq!(t.observe(3), SeqEvent::Restart);
        assert_eq!(t.observe(4), SeqEvent::InOrder);
    }

    #[test]
    fn sequence_tracker_handles_u32_wrap() {
        let mut t = SequenceTracker::default();
        t.observe(u32::MAX - 1);
        assert_eq!(t.observe(u32::MAX), SeqEvent::InOrder);
        assert_eq!(t.observe(0), SeqEvent::InOrder);
    }
}
