use std::ffi::c_void;

use arcp_core::{
    SignalAccess, SignalDescriptor, SignalDurability, SignalKind, SignalPolicy, SignalType,
};
use arcp_server::{ArcpServer, ArcpServerConfig};

#[repr(C)]
pub struct ArcpRuntimeHandle {
    pub(crate) server: ArcpServer,
}

pub(crate) fn runtime_create() -> *mut ArcpRuntimeHandle {
    let runtime = ArcpRuntimeHandle {
        server: ArcpServer::new(ArcpServerConfig::default()),
    };
    Box::into_raw(Box::new(runtime))
}

pub(crate) fn runtime_destroy(handle: *mut ArcpRuntimeHandle) {
    if handle.is_null() {
        return;
    }
    // SAFETY: pointer was created by Box::into_raw in runtime_create.
    unsafe {
        drop(Box::from_raw(handle));
    }
}

pub(crate) fn runtime_start(handle: *mut ArcpRuntimeHandle) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.start() {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn runtime_stop(handle: *mut ArcpRuntimeHandle) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    runtime.server.stop();
    0
}

pub(crate) fn runtime_control_port(handle: *mut ArcpRuntimeHandle) -> u16 {
    if handle.is_null() {
        return 0;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    runtime.server.control_port()
}

pub(crate) fn runtime_realtime_port(handle: *mut ArcpRuntimeHandle) -> u16 {
    if handle.is_null() {
        return 0;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    runtime.server.realtime_port()
}

pub(crate) fn register_signal(
    handle: *mut ArcpRuntimeHandle,
    signal_id: u16,
    signal_type: SignalType,
    signal_kind: SignalKind,
    signal_access: SignalAccess,
    signal_policy: SignalPolicy,
    signal_durability: SignalDurability,
    path: &str,
) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.register_signal(SignalDescriptor::new(
        signal_id,
        signal_type,
        signal_kind,
        signal_access,
        signal_policy,
        signal_durability,
        path,
    )) {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn publish_bool(handle: *mut ArcpRuntimeHandle, signal_id: u16, value: bool) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.publish_bool(signal_id, value) {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn publish_i64(handle: *mut ArcpRuntimeHandle, signal_id: u16, value: i64) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.publish_i64(signal_id, value) {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn publish_f64(handle: *mut ArcpRuntimeHandle, signal_id: u16, value: f64) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.publish_f64(signal_id, value) {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn publish_string(handle: *mut ArcpRuntimeHandle, signal_id: u16, value: String) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.publish_string(signal_id, value) {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn publish_bool_array(
    handle: *mut ArcpRuntimeHandle,
    signal_id: u16,
    values: Vec<bool>,
) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.publish_bool_array(signal_id, values) {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn publish_i64_array(
    handle: *mut ArcpRuntimeHandle,
    signal_id: u16,
    values: Vec<i64>,
) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.publish_i64_array(signal_id, values) {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn publish_f64_array(
    handle: *mut ArcpRuntimeHandle,
    signal_id: u16,
    values: Vec<f64>,
) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.publish_f64_array(signal_id, values) {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn publish_string_array(
    handle: *mut ArcpRuntimeHandle,
    signal_id: u16,
    values: Vec<String>,
) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    match runtime.server.publish_string_array(signal_id, values) {
        Ok(()) => 0,
        Err(_) => -2,
    }
}

pub(crate) fn poll_events_into_slice(handle: *mut ArcpRuntimeHandle, out_slice: &mut [u8]) -> i32 {
    if handle.is_null() {
        return -1;
    }
    // SAFETY: null-checked above; caller owns lifetime.
    let runtime = unsafe { &mut *handle };
    runtime.server.poll_events_into(out_slice) as i32
}

#[no_mangle]
pub extern "C" fn arcp_runtime_create() -> *mut ArcpRuntimeHandle {
    runtime_create()
}

#[no_mangle]
pub extern "C" fn arcp_runtime_destroy(handle: *mut ArcpRuntimeHandle) {
    runtime_destroy(handle)
}

#[no_mangle]
pub extern "C" fn arcp_runtime_start(handle: *mut ArcpRuntimeHandle) -> i32 {
    runtime_start(handle)
}

#[no_mangle]
pub extern "C" fn arcp_runtime_stop(handle: *mut ArcpRuntimeHandle) -> i32 {
    runtime_stop(handle)
}

#[no_mangle]
pub extern "C" fn arcp_runtime_control_port(handle: *mut ArcpRuntimeHandle) -> u16 {
    runtime_control_port(handle)
}

#[no_mangle]
pub extern "C" fn arcp_runtime_realtime_port(handle: *mut ArcpRuntimeHandle) -> u16 {
    runtime_realtime_port(handle)
}

#[no_mangle]
pub extern "C" fn arcp_register_signal(
    handle: *mut ArcpRuntimeHandle,
    signal_id: u16,
    signal_type: u8,
    signal_kind: u8,
    signal_access: u8,
    signal_policy: u8,
    signal_durability: u8,
    path: *const u8,
    path_len: u32,
) -> i32 {
    if path.is_null() {
        return -1;
    }
    let signal_type = match SignalType::try_from(signal_type) {
        Ok(signal_type) => signal_type,
        Err(_) => return -3,
    };
    let signal_kind = match SignalKind::try_from(signal_kind) {
        Ok(signal_kind) => signal_kind,
        Err(_) => return -4,
    };
    let signal_access = match SignalAccess::try_from(signal_access) {
        Ok(signal_access) => signal_access,
        Err(_) => return -5,
    };
    let signal_policy = match SignalPolicy::try_from(signal_policy) {
        Ok(signal_policy) => signal_policy,
        Err(_) => return -6,
    };
    let signal_durability = match SignalDurability::try_from(signal_durability) {
        Ok(signal_durability) => signal_durability,
        Err(_) => return -7,
    };
    // SAFETY: caller provides a valid pointer of length path_len.
    let raw_path = unsafe { std::slice::from_raw_parts(path, path_len as usize) };
    let path = match std::str::from_utf8(raw_path) {
        Ok(path) => path,
        Err(_) => return -8,
    };
    register_signal(
        handle,
        signal_id,
        signal_type,
        signal_kind,
        signal_access,
        signal_policy,
        signal_durability,
        path,
    )
}

#[no_mangle]
pub extern "C" fn arcp_publish_bool(
    handle: *mut ArcpRuntimeHandle,
    signal_id: u16,
    value: bool,
) -> i32 {
    publish_bool(handle, signal_id, value)
}

#[no_mangle]
pub extern "C" fn arcp_publish_i64(
    handle: *mut ArcpRuntimeHandle,
    signal_id: u16,
    value: i64,
) -> i32 {
    publish_i64(handle, signal_id, value)
}

#[no_mangle]
pub extern "C" fn arcp_publish_f64(
    handle: *mut ArcpRuntimeHandle,
    signal_id: u16,
    value: f64,
) -> i32 {
    publish_f64(handle, signal_id, value)
}

#[no_mangle]
pub extern "C" fn arcp_poll_events(
    handle: *mut ArcpRuntimeHandle,
    out_buffer: *mut c_void,
    out_capacity: u32,
) -> i32 {
    if out_buffer.is_null() {
        return -1;
    }
    // SAFETY: caller provides a valid writable pointer with out_capacity bytes.
    let out_slice =
        unsafe { std::slice::from_raw_parts_mut(out_buffer as *mut u8, out_capacity as usize) };
    poll_events_into_slice(handle, out_slice)
}
