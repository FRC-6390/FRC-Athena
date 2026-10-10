//! `extern "system"` entry points for `ca.frc6390.athena.arcp.ArcpNative`.
//!
//! Every call is synchronous and allocation-light: buffers are direct `ByteBuffer`s owned by
//! Java, read or written in place. A failed call throws `IllegalStateException` and returns the
//! fallback value.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use arcp_core::v2::record::Marker;
use arcp_server::v2::{ArcpServerV2, RobotHandler, ServerConfig};
use jni::objects::{JByteBuffer, JClass, JLongArray, JString};
use jni::strings::JNIString;
use jni::sys::{jint, jlong, jlongArray, jstring};
use jni::{jni_str, Env, EnvUnowned, Outcome};

use crate::bridge::{Bridge, BridgeHandler};
use crate::marshal::Reader;

/// A running server plus the queue its robot-thread requests go through.
pub struct NativeServer {
    server: ArcpServerV2,
    bridge: Arc<Bridge>,
}

const MTU_MIN: i32 = 128;
const MTU_MAX: i32 = 65_507;

/// Runs `f` with the JNI env. Errors and panics become a Java exception and `fallback`.
fn run<T: Copy>(
    env: &mut EnvUnowned<'_>,
    fallback: T,
    f: impl FnOnce(&mut Env<'_>) -> Result<T, String>,
) -> T {
    let outcome = env
        .with_env(|env| -> jni::errors::Result<T> {
            match f(env) {
                Ok(value) => Ok(value),
                Err(message) => {
                    let _ = env.throw_new(
                        jni_str!("java/lang/IllegalStateException"),
                        JNIString::from(message),
                    );
                    Ok(fallback)
                }
            }
        })
        .into_outcome();
    match outcome {
        Outcome::Ok(value) => value,
        Outcome::Err(_) | Outcome::Panic(_) => fallback,
    }
}

fn server<'a>(handle: jlong) -> Result<&'a NativeServer, String> {
    if handle == 0 {
        return Err("ARCP server is closed".into());
    }
    // SAFETY: `handle` was produced by `create` and stays valid until Java calls `destroy`
    // exactly once (the Java side guards this with a closed flag).
    Ok(unsafe { &*(handle as *const NativeServer) })
}

/// Borrows the first `len` bytes of a direct buffer for the duration of the call.
fn buffer<'a>(env: &Env<'_>, buf: &JByteBuffer<'_>, len: jint) -> Result<&'a mut [u8], String> {
    let capacity = env
        .get_direct_buffer_capacity(buf)
        .map_err(|e| e.to_string())?;
    let address = env
        .get_direct_buffer_address(buf)
        .map_err(|e| e.to_string())?;
    if address.is_null() {
        return Err("buffer is not a direct ByteBuffer".into());
    }
    if len < 0 || len as usize > capacity {
        return Err(format!("buffer length {len} exceeds capacity {capacity}"));
    }
    // SAFETY: the direct buffer stays alive and unmodified by Java while this native call runs,
    // and `len` was checked against its capacity.
    Ok(unsafe { std::slice::from_raw_parts_mut(address, len as usize) })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_create<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    control_port: jint,
    realtime_port: jint,
    file_root: JString<'local>,
    budget_bytes_per_second: jlong,
    mtu: jint,
    keyframe_interval_micros: jlong,
    file_categories: JString<'local>,
) -> jlong {
    run(&mut env, 0, |env| {
        if !(0..=65_535).contains(&control_port) {
            return Err(format!("control port {control_port} out of range"));
        }
        if !(0..=65_535).contains(&realtime_port) {
            return Err(format!("realtime port {realtime_port} out of range"));
        }
        if !(MTU_MIN..=MTU_MAX).contains(&mtu) {
            return Err(format!("mtu {mtu} out of range"));
        }
        let file_root: Option<PathBuf> = if file_root.is_null() {
            None
        } else {
            Some(PathBuf::from(
                file_root.try_to_string(env).map_err(|e| e.to_string())?,
            ))
        };
        let file_categories = if file_categories.is_null() {
            Vec::new()
        } else {
            parse_categories(&file_categories.try_to_string(env).map_err(|e| e.to_string())?)?
        };
        let config = ServerConfig {
            control_bind: SocketAddr::from(([0, 0, 0, 0], control_port as u16)),
            realtime_port: realtime_port as u16,
            file_root,
            file_categories,
            bandwidth_budget_bytes_per_sec: budget_bytes_per_second.max(0) as u64,
            mtu: mtu as usize,
            keyframe_interval: std::time::Duration::from_micros(
                keyframe_interval_micros.max(1) as u64
            ),
        };
        let bridge = Bridge::new();
        let handler: Arc<dyn RobotHandler> = Arc::new(BridgeHandler(Arc::clone(&bridge)));
        let server = ArcpServerV2::start(config, handler)
            .map_err(|e| format!("cannot start ARCP server: {e}"))?;
        Ok(Box::into_raw(Box::new(NativeServer { server, bridge })) as jlong)
    })
}

/// Parses `name<TAB>directory` lines. Names must be plain; directories must be absolute.
fn parse_categories(text: &str) -> Result<Vec<(String, PathBuf)>, String> {
    let mut out = Vec::new();
    for line in text.lines().filter(|l| !l.is_empty()) {
        let (name, dir) = line
            .split_once('\t')
            .ok_or_else(|| format!("bad file category line: {line}"))?;
        if name.is_empty() || name.contains(['/', '\\', ':']) || name == "." || name == ".." {
            return Err(format!("bad file category name: {name}"));
        }
        let dir = PathBuf::from(dir);
        if !dir.is_absolute() {
            return Err(format!("file category directory must be absolute: {dir:?}"));
        }
        out.push((name.to_owned(), dir));
    }
    Ok(out)
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_destroy<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
) {
    run(&mut env, (), |_| {
        if handle != 0 {
            // SAFETY: called once per handle by Java; the pointer came from `Box::into_raw` in `create`.
            let native = unsafe { Box::from_raw(handle as *mut NativeServer) };
            native.server.shutdown();
        }
        Ok(())
    });
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_controlPort<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
) -> jint {
    run(&mut env, -1, |_| {
        Ok(i32::from(server(handle)?.server.control_addr().port()))
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_realtimePort<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
) -> jint {
    run(&mut env, -1, |_| {
        Ok(i32::from(server(handle)?.server.realtime_addr().port()))
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishDictionary<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
    buf: JByteBuffer<'local>,
    len: jint,
) {
    run(&mut env, (), |env| {
        let native = server(handle)?;
        let bytes = buffer(env, &buf, len)?;
        let (dictionary, schemas) = Reader::new(bytes).dictionary()?;
        native.bridge.set_dictionary(&dictionary);
        native.server.publish_dictionary(dictionary, schemas);
        Ok(())
    });
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishCycle<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
    timestamp_micros: jlong,
    buf: JByteBuffer<'local>,
    len: jint,
) {
    run(&mut env, (), |env| {
        let native = server(handle)?;
        let bytes = buffer(env, &buf, len)?;
        let entries = Reader::new(bytes).cycle()?;
        native.bridge.note_cycle(&entries);
        native
            .server
            .publish_cycle(timestamp_micros.max(0) as u64, &entries);
        Ok(())
    });
}

/// Sends raw replay record envelopes (dictionary, frames, markers) to replay subscribers.
#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_publishRecords<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
    buf: JByteBuffer<'local>,
    len: jint,
) {
    run(&mut env, (), |env| {
        let native = server(handle)?;
        let bytes = buffer(env, &buf, len)?;
        native.server.publish_records(bytes);
        Ok(())
    });
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_marker<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
    timestamp_micros: jlong,
    kind: jint,
    signal_id: jlong,
    message: JString<'local>,
) {
    run(&mut env, (), |env| {
        let native = server(handle)?;
        let message = message.try_to_string(env).map_err(|e| e.to_string())?;
        native.server.marker(Marker {
            timestamp_micros,
            kind: kind.max(0) as u64,
            message,
            signal_id: signal_id.max(0) as u64,
        });
        Ok(())
    });
}

/// Fills `buf` with pending client requests. Returns the bytes written (0 when none), or the
/// negative size needed when the first pending request does not fit.
#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_poll<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
    buf: JByteBuffer<'local>,
    capacity: jint,
) -> jint {
    run(&mut env, -1, |env| {
        let native = server(handle)?;
        let bytes = buffer(env, &buf, capacity)?;
        Ok(native.bridge.poll(bytes))
    })
}

#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_reply<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
    buf: JByteBuffer<'local>,
    len: jint,
) {
    run(&mut env, (), |env| {
        let native = server(handle)?;
        let bytes = buffer(env, &buf, len)?;
        native.bridge.apply_replies(bytes)
    });
}

/// `[sessions, subscriptions, bytesLastWindow, throttledSubscriptions]`.
#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_status<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
) -> jlongArray {
    let null: jlongArray = std::ptr::null_mut();
    run(&mut env, null, |env| {
        let status = server(handle)?.server.status();
        let values: [jlong; 4] = [
            status.sessions as jlong,
            status.subscriptions as jlong,
            status.bytes_last_window as jlong,
            status.throttled.len() as jlong,
        ];
        let array: JLongArray = env
            .new_long_array(values.len())
            .map_err(|e| e.to_string())?;
        array
            .set_region(env, 0, &values)
            .map_err(|e| e.to_string())?;
        Ok(array.into_raw())
    })
}

/// Throttled subscriptions as `session/subscription pattern requested-rate divisor`, one per line.
#[no_mangle]
pub extern "system" fn Java_ca_frc6390_athena_arcp_ArcpNative_throttled<'local>(
    mut env: EnvUnowned<'local>,
    _class: JClass<'local>,
    handle: jlong,
) -> jstring {
    let null: jstring = std::ptr::null_mut();
    run(&mut env, null, |env| {
        let status = server(handle)?.server.status();
        let lines: Vec<String> = status
            .throttled
            .iter()
            .map(|t| {
                format!(
                    "{}/{} {} {}",
                    t.session_id, t.subscription_id, t.pattern, t.divisor
                )
            })
            .collect();
        let text = env
            .new_string(lines.join("\n"))
            .map_err(|e| e.to_string())?;
        Ok(text.into_raw())
    })
}
