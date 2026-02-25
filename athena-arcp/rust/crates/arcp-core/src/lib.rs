//! Core protocol model for ARCP.

mod codec;
mod constants;
mod types;

pub use codec::{
    decode_event, decode_update, encode_event, encode_event_into, encode_update,
    encode_update_into,
};
pub use constants::ARCP_PROTOCOL_VERSION;
pub use types::{
    PersistenceScope, RuntimeEvent, SignalAccess, SignalDescriptor, SignalDurability, SignalKind,
    SignalPolicy, SignalType, SignalValue,
};
