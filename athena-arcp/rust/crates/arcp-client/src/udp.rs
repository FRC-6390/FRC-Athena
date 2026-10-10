//! UDP realtime receive loop: decode datagrams, reassemble frames, track sequence gaps and
//! request backfill for them.

use std::net::UdpSocket;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;

use arcp_core::v2::record::{decode_stream, Record};

use crate::client::{backfill_records, lock, Event, Shared};
use crate::reassembly::{Reassembler, SeqEvent, SequenceTracker};

pub(crate) fn run(shared: Arc<Shared>, socket: UdpSocket) {
    let mut seq = SequenceTracker::default();
    let mut reassembly = Reassembler::default();
    let mut buf = vec![0u8; 65_536];
    while !shared.shutdown.load(Ordering::SeqCst) {
        match socket.recv_from(&mut buf) {
            Ok((n, _)) => handle_datagram(&shared, &mut seq, &mut reassembly, &buf[..n]),
            // Read timeouts (used to notice shutdown) and transient errors both land here.
            Err(_) => {}
        }
    }
}

fn handle_datagram(shared: &Arc<Shared>, seq: &mut SequenceTracker, reassembly: &mut Reassembler, data: &[u8]) {
    let records = match decode_stream(data) {
        Ok(records) => records,
        Err(_) => {
            shared.counters.malformed.fetch_add(1, Ordering::Relaxed);
            return;
        }
    };
    for record in records {
        match record {
            Record::Frame(frame) => {
                for complete in reassembly.push(frame) {
                    deliver_frame(shared, seq, complete);
                }
                shared.counters.dropped_incomplete.fetch_add(reassembly.take_dropped(), Ordering::Relaxed);
            }
            Record::Dictionary(dictionary) => {
                let schemas = {
                    let mut model = lock(&shared.model);
                    let schemas = model.schemas().to_vec();
                    model.set_dictionary(dictionary.clone(), schemas.clone());
                    schemas
                };
                let _ = shared.events.send(Event::Dictionary { dictionary, schemas });
            }
            Record::StructSchema(schema) => {
                lock(&shared.model).upsert_schema(schema.clone());
                let _ = shared.events.send(Event::Record(Record::StructSchema(schema)));
            }
            other => {
                let _ = shared.events.send(Event::Record(other));
            }
        }
    }
}

fn deliver_frame(shared: &Arc<Shared>, seq: &mut SequenceTracker, frame: arcp_core::v2::record::Frame) {
    match seq.observe(frame.sequence) {
        SeqEvent::Duplicate => {
            shared.counters.duplicates.fetch_add(1, Ordering::Relaxed);
            return;
        }
        SeqEvent::Gap { from, to, lost } => {
            shared.counters.gaps.fetch_add(1, Ordering::Relaxed);
            shared.counters.lost_cycles.fetch_add(u64::from(lost), Ordering::Relaxed);
            let _ = shared.events.send(Event::Gap { from, to, lost });
            if shared.auto_backfill {
                let shared = Arc::clone(shared);
                let (from, to) = (u64::from(from), u64::from(to));
                thread::Builder::new()
                    .name("arcp-client-backfill".into())
                    .spawn(move || {
                        let result = backfill_records(&shared, from, to).map_err(|e| e.to_string());
                        if result.is_ok() {
                            shared.counters.backfills.fetch_add(1, Ordering::Relaxed);
                        }
                        let _ = shared.events.send(Event::Backfill { from, to, result });
                    })
                    .ok();
            }
        }
        SeqEvent::Restart => {
            shared.counters.restarts.fetch_add(1, Ordering::Relaxed);
        }
        SeqEvent::First | SeqEvent::InOrder => {}
    }
    shared.counters.frames.fetch_add(1, Ordering::Relaxed);
    {
        let mut model = lock(&shared.model);
        for entry in &frame.entries {
            model.set_value(entry.signal_id, entry.value.clone());
        }
    }
    let _ = shared.events.send(Event::Frame(frame));
}
