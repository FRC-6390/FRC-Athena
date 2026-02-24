//! Core protocol model for ARCP.

mod codec;
mod constants;
mod types;

pub use codec::{decode_event, decode_update, encode_event, encode_update};
pub use constants::ARCP_PROTOCOL_VERSION;
pub use types::{
    PersistenceScope, RuntimeEvent, SignalAccess, SignalDescriptor, SignalDurability, SignalKind,
    SignalPolicy, SignalType, SignalValue,
};
