use jni::objects::{JBooleanArray, JClass, JDoubleArray, JLongArray, JObjectArray, JString};
use jni::sys::{jboolean, jdouble, jint, jlong};
use jni::{EnvUnowned, Outcome};

use crate::runtime::{
    publish_bool, publish_bool_array, publish_f64, publish_f64_array, publish_i64,
    publish_i64_array, publish_string, publish_string_array, ArcpRuntimeHandle,
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

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishBoolean(
    _env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    signal_id: jint,
    value: jboolean,
) -> jint {
    if signal_id <= 0 || signal_id > u16::MAX as jint {
        return -3;
    }
    publish_bool(handle as *mut ArcpRuntimeHandle, signal_id as u16, value) as jint
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishI64(
    _env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    signal_id: jint,
    value: jlong,
) -> jint {
    if signal_id <= 0 || signal_id > u16::MAX as jint {
        return -3;
    }
    publish_i64(handle as *mut ArcpRuntimeHandle, signal_id as u16, value) as jint
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishF64(
    _env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    signal_id: jint,
    value: jdouble,
) -> jint {
    if signal_id <= 0 || signal_id > u16::MAX as jint {
        return -3;
    }
    publish_f64(handle as *mut ArcpRuntimeHandle, signal_id as u16, value) as jint
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishString(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    signal_id: jint,
    value: JString,
) -> jint {
    if signal_id <= 0 || signal_id > u16::MAX as jint {
        return -3;
    }

    with_env_code(&mut env, -5, |env| {
        let text = match value.try_to_string(env) {
            Ok(text) => text,
            Err(_) => return -5,
        };
        publish_string(handle as *mut ArcpRuntimeHandle, signal_id as u16, text) as jint
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishBooleanArray(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    signal_id: jint,
    array: JBooleanArray,
) -> jint {
    if signal_id <= 0 || signal_id > u16::MAX as jint {
        return -3;
    }

    with_env_code(&mut env, -6, |env| {
        let len = match array.len(env) {
            Ok(len) => len,
            Err(_) => return -6,
        };
        let mut values = vec![false; len];
        if array.get_region(env, 0, &mut values).is_err() {
            return -6;
        }
        publish_bool_array(handle as *mut ArcpRuntimeHandle, signal_id as u16, values) as jint
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishI64Array(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    signal_id: jint,
    array: JLongArray,
) -> jint {
    if signal_id <= 0 || signal_id > u16::MAX as jint {
        return -3;
    }

    with_env_code(&mut env, -6, |env| {
        let len = match array.len(env) {
            Ok(len) => len,
            Err(_) => return -6,
        };
        let mut values = vec![0_i64; len];
        if array.get_region(env, 0, &mut values).is_err() {
            return -6;
        }
        publish_i64_array(handle as *mut ArcpRuntimeHandle, signal_id as u16, values) as jint
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishF64Array(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    signal_id: jint,
    array: JDoubleArray,
) -> jint {
    if signal_id <= 0 || signal_id > u16::MAX as jint {
        return -3;
    }

    with_env_code(&mut env, -6, |env| {
        let len = match array.len(env) {
            Ok(len) => len,
            Err(_) => return -6,
        };
        let mut values = vec![0_f64; len];
        if array.get_region(env, 0, &mut values).is_err() {
            return -6;
        }
        publish_f64_array(handle as *mut ArcpRuntimeHandle, signal_id as u16, values) as jint
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishStringArray(
    mut env: EnvUnowned,
    _class: JClass,
    handle: jlong,
    signal_id: jint,
    array: JObjectArray,
) -> jint {
    if signal_id <= 0 || signal_id > u16::MAX as jint {
        return -3;
    }

    with_env_code(&mut env, -6, |env| {
        let len = match array.len(env) {
            Ok(len) => len,
            Err(_) => return -6,
        };
        let mut values = Vec::with_capacity(len);

        for index in 0..len {
            let obj = match array.get_element(env, index) {
                Ok(obj) => obj,
                Err(_) => return -6,
            };
            let str_obj = match env.cast_local::<JString>(obj) {
                Ok(str_obj) => str_obj,
                Err(_) => return -6,
            };
            let text = match str_obj.try_to_string(env) {
                Ok(text) => text,
                Err(_) => return -6,
            };
            values.push(text);
        }

        publish_string_array(handle as *mut ArcpRuntimeHandle, signal_id as u16, values) as jint
    })
}
