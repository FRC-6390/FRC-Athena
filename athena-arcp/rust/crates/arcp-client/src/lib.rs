//! ARCP v2 client (dashboard data core, F4).
//!
//! * Control: a reliable TCP request/reply channel with request ids (subscribe, write, action,
//!   time sync, files, replay subscribe, backfill).
//! * Realtime: UDP frames, reassembled across fragments, deduplicated and checked for sequence
//!   gaps. A gap triggers a backfill request over the control channel.
//! * Model: the dictionary and schemas from the robot, plus the latest value of every signal.
//!
//! Everything runs on plain threads and delivers [`Event`]s over an `mpsc` channel so the Tauri
//! layer can batch them for the UI.

mod client;
mod control;
mod model;
mod reassembly;
mod timesync;
mod udp;
pub mod ws_bridge;

pub use arcp_core::v2::control::{FileInfo, Rate, Status, WriteReply, MAX_FILE_CHUNK};
pub use arcp_core::v2::record::{DictEntry, Dictionary, Frame, Marker, Record, StructSchema};
pub use arcp_core::v2::{Entry, Value};
pub use client::{decode_backfill, Client, ClientConfig, ClientError, Event, LinkStatus, DEFAULT_CONTROL_PORT, DEFAULT_UDP_PORT};
pub use model::{Model, ServerStatus};
pub use reassembly::{Reassembler, SeqEvent, SequenceTracker};
pub use timesync::{estimate, TimeEstimate, TimeSample};
