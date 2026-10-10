//! Core protocol model for ARCP.
//!
//! The original v1 model (`codec`, `types`) remains for `arcp-server` v1 and `arcp-jni`.
//! The v2 wire format used by the off-season runtime lives in [`v2`].

mod codec;
mod constants;
mod types;

pub mod v2;

pub use codec::{
    decode_event, decode_update, encode_event, encode_event_into, encode_update,
    encode_update_into,
};
pub use constants::ARCP_PROTOCOL_VERSION;
pub use types::{
    PersistenceScope, RuntimeEvent, SignalAccess, SignalDescriptor, SignalDurability, SignalKind,
    SignalPolicy, SignalType, SignalValue,
};
