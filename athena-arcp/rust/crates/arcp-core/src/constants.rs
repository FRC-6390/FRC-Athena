pub const ARCP_PROTOCOL_VERSION: u8 = 1;

pub(crate) const FRAME_BOOL: u8 = 1;
pub(crate) const FRAME_I64: u8 = 2;
pub(crate) const FRAME_F64: u8 = 3;
pub(crate) const FRAME_STR: u8 = 4;
pub(crate) const FRAME_BOOL_ARRAY: u8 = 5;
pub(crate) const FRAME_I64_ARRAY: u8 = 6;
pub(crate) const FRAME_F64_ARRAY: u8 = 7;
pub(crate) const FRAME_STR_ARRAY: u8 = 8;

pub(crate) const EVENT_TUNABLE_SET: u8 = 1;
pub(crate) const EVENT_ACTION: u8 = 2;
