use std::ffi::c_void;
use std::ptr;

use arcp_core::{SignalAccess, SignalDurability, SignalKind, SignalPolicy, SignalType};
use jni::objects::{JByteBuffer, JClass, JObject, JObjectArray, JString};
use jni::sys::{jint, jlong, jobjectArray, jstring};
use jni::{EnvUnowned, Outcome};

use crate::runtime::{
    list_layouts, load_layout, poll_events_into_slice, register_signal, runtime_control_port,
    runtime_create, runtime_destroy, runtime_realtime_port, runtime_start, runtime_stop,
    save_layout, ArcpRuntimeHandle,
};

fn with_env_code(
    env: &mut EnvUnowned<'_>,
    fallback: jint,
    f: impl FnOnce(&mut jni::Env<'_>) -> jint,
) -> jint {
    match env
        .with_env(|env| -> jni::errors::Result<jint> { Ok(f(env)) })
        .into_outcome()
    {
        Outcome::Ok(code) => code,
        Outcome::Err(_) | Outcome::Panic(_) => fallback,
    }
}

fn with_env_string(
    env: &mut EnvUnowned<'_>,
    fallback: jstring,
    f: impl FnOnce(&mut jni::Env<'_>) -> jstring,
) -> jstring {
    match env
        .with_env(|env| -> jni::errors::Result<jstring> { Ok(f(env)) })
        .into_outcome()
    {
        Outcome::Ok(value) => value,
        Outcome::Err(_) | Outcome::Panic(_) => fallback,
    }
}

fn with_env_object_array(
    env: &mut EnvUnowned<'_>,
    fallback: jobjectArray,
    f: impl FnOnce(&mut jni::Env<'_>) -> jobjectArray,
) -> jobjectArray {
    match env
        .with_env(|env| -> jni::errors::Result<jobjectArray> { Ok(f(env)) })
        .into_outcome()
    {
        Outcome::Ok(value) => value,
        Outcome::Err(_) | Outcome::Panic(_) => fallback,
    }
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_runtimeCreate(
    _env: EnvUnowned,
    _class: JClass,
) -> jlong {
    runtime_create() as jlong
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_runtimeDestroy(
    _env: EnvUnowned,
    _class: JClass,
    handle: jlong,
) {
    runtime_destroy(handle as *mut ArcpRuntimeHandle);
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_runtimeStart(
    _env: EnvUnowned,
    _class: JClass,
    handle: jlong,
) -> jint {
    runtime_start(handle as *mut ArcpRuntimeHandle) as jint
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_runtimeStop(
    _env: EnvUnowned,
    _class: JClass,
    handle: jlong,
) -> jint {
    runtime_stop(handle as *mut ArcpRuntimeHandle) as jint
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_controlPort(
    _env: EnvUnowned,
    _class: JClass,
    handle: jlong,
) -> jint {
    runtime_control_port(handle as *mut ArcpRuntimeHandle) as jint
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_realtimePort(
    _env: EnvUnowned,
    _class: JClass,
    handle: jlong,
) -> jint {
    runtime_realtime_port(handle as *mut ArcpRuntimeHandle) as jint
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_registerSignal(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    signal_id: jint,
    signal_type: jint,
    signal_kind: jint,
    signal_access: jint,
    signal_policy: jint,
    signal_durability: jint,
    path: JString,
) -> jint {
    if signal_id <= 0 || signal_id > u16::MAX as jint {
        return -3;
    }
    let signal_type = match SignalType::try_from(signal_type as u8) {
        Ok(signal_type) => signal_type,
        Err(_) => return -3,
    };
    let signal_kind = match SignalKind::try_from(signal_kind as u8) {
        Ok(signal_kind) => signal_kind,
        Err(_) => return -4,
    };
    let signal_access = match SignalAccess::try_from(signal_access as u8) {
        Ok(signal_access) => signal_access,
        Err(_) => return -5,
    };
    let signal_policy = match SignalPolicy::try_from(signal_policy as u8) {
        Ok(signal_policy) => signal_policy,
        Err(_) => return -6,
    };
    let signal_durability = match SignalDurability::try_from(signal_durability as u8) {
        Ok(signal_durability) => signal_durability,
        Err(_) => return -7,
    };

    with_env_code(&mut env, -8, |env| {
        let path_string = match path.try_to_string(env) {
            Ok(path) => path,
            Err(_) => return -8,
        };

        register_signal(
            handle as *mut ArcpRuntimeHandle,
            signal_id as u16,
            signal_type,
            signal_kind,
            signal_access,
            signal_policy,
            signal_durability,
            &path_string,
        ) as jint
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_pollEvents(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    out_buffer: JObject,
    out_capacity: jint,
) -> jint {
    if out_capacity <= 0 {
        return 0;
    }

    with_env_code(&mut env, -1, |env| {
        let byte_buffer = match env.cast_local::<JByteBuffer>(out_buffer) {
            Ok(buffer) => buffer,
            Err(_) => return -1,
        };
        let raw_ptr = match env.get_direct_buffer_address(&byte_buffer) {
            Ok(buf) => buf as *mut c_void,
            Err(_) => ptr::null_mut(),
        };
        if raw_ptr.is_null() {
            return -1;
        }

        // SAFETY: JNI returned a valid direct buffer pointer and writes are bounded by out_capacity.
        let out_slice =
            unsafe { std::slice::from_raw_parts_mut(raw_ptr as *mut u8, out_capacity as usize) };
        poll_events_into_slice(handle as *mut ArcpRuntimeHandle, out_slice) as jint
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_saveLayout(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    name: JString,
    layout_json: JString,
) -> jint {
    with_env_code(&mut env, -8, |env| {
        let name_string = match name.try_to_string(env) {
            Ok(value) => value,
            Err(_) => return -8,
        };
        let layout_string = match layout_json.try_to_string(env) {
            Ok(value) => value,
            Err(_) => return -8,
        };
        save_layout(
            handle as *mut ArcpRuntimeHandle,
            &name_string,
            &layout_string,
        ) as jint
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_loadLayout(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    name: JString,
) -> jstring {
    with_env_string(&mut env, ptr::null_mut(), |env| {
        let name_string = match name.try_to_string(env) {
            Ok(value) => value,
            Err(_) => return ptr::null_mut(),
        };
        let payload = match load_layout(handle as *mut ArcpRuntimeHandle, &name_string) {
            Ok(payload) => payload,
            Err(_) => return ptr::null_mut(),
        };
        match JString::from_str(env, payload) {
            Ok(value) => {
                let raw: JObject = value.into();
                raw.into_raw() as jstring
            }
            Err(_) => ptr::null_mut(),
        }
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_listLayouts(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
) -> jobjectArray {
    with_env_object_array(&mut env, ptr::null_mut(), |env| {
        let names = match list_layouts(handle as *mut ArcpRuntimeHandle) {
            Ok(names) => names,
            Err(_) => return ptr::null_mut(),
        };
        let seed = match JString::from_str(env, "") {
            Ok(seed) => seed,
            Err(_) => return ptr::null_mut(),
        };
        let array = match JObjectArray::<JString>::new(env, names.len(), &seed) {
            Ok(array) => array,
            Err(_) => return ptr::null_mut(),
        };
        for (index, name) in names.into_iter().enumerate() {
            let value = match JString::from_str(env, name) {
                Ok(value) => value,
                Err(_) => return ptr::null_mut(),
            };
            if array.set_element(env, index, &value).is_err() {
                return ptr::null_mut();
            }
        }
        let raw: JObject = array.into();
        raw.into_raw() as jobjectArray
    })
}
