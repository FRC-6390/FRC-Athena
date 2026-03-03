use std::collections::HashMap;
use std::io;
use std::net::{IpAddr, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use arcp_core::{
    PersistenceScope, SignalAccess, SignalDurability, SignalKind, SignalPolicy, SignalType,
    SignalValue,
};
use arcp_dashboard::{DashboardState, ManifestItem};
use nt_client::data::r#type::{DataType, NetworkTableData};
use nt_client::data::SubscriptionOptions;
use nt_client::subscribe::ReceivedMessage;
use nt_client::topic::AnnouncedTopic;
use nt_client::{Client, NTAddr, NewClientOptions};

const NT4_UNSECURE_PORT_DEFAULT: u16 = 5810;
const NT4_MAX_MIRRORED_TOPICS: usize = 2048;
const NT4_SIGNAL_ID_START: u16 = 65535;
const NT4_SIGNAL_ID_MIN: u16 = 50000;
const NT4_DIAG_CONNECTED_ID: u16 = 49999;
const NT4_DIAG_TOPIC_COUNT_ID: u16 = 49998;
const NT4_DIAG_LAST_ERROR_ID: u16 = 49997;

#[derive(Clone, Copy, Debug)]
struct Nt4Binding {
    signal_id: u16,
    signal_type: SignalType,
}

pub fn spawn_nt4_client_worker(
    running: Arc<AtomicBool>,
    state: Arc<Mutex<DashboardState>>,
    host: String,
    unsecure_port: Option<u16>,
) -> io::Result<JoinHandle<()>> {
    let unsecure_port = unsecure_port.unwrap_or(NT4_UNSECURE_PORT_DEFAULT);
    thread::Builder::new()
        .name("arcp-dashboard-nt4".to_string())
        .spawn(move || run_nt4_client_loop(running, state, host, unsecure_port))
}

fn run_nt4_client_loop(
    running: Arc<AtomicBool>,
    state: Arc<Mutex<DashboardState>>,
    host: String,
    unsecure_port: u16,
) {
    init_nt4_diagnostics(&state);

    let runtime = match tokio::runtime::Builder::new_current_thread()
        .enable_io()
        .enable_time()
        .build()
    {
        Ok(runtime) => runtime,
        Err(_) => return,
    };

    runtime.block_on(async move {
        let mut topic_bindings: HashMap<String, Nt4Binding> = HashMap::new();
        let mut last_values: HashMap<u16, SignalValue> = HashMap::new();
        let mut next_signal_id = NT4_SIGNAL_ID_START;
        set_nt4_diagnostics(&state, false, 0, "resolving host");

        while running.load(Ordering::SeqCst) {
            let Some(addr) = resolve_host_ipv4(&host) else {
                set_nt4_diagnostics(
                    &state,
                    false,
                    topic_bindings.len(),
                    "failed to resolve host",
                );
                tokio::time::sleep(Duration::from_millis(500)).await;
                continue;
            };

            let options = NewClientOptions {
                addr: NTAddr::Custom(addr),
                unsecure_port,
                secure_port: None,
                name: "arcp-host-nt4".to_string(),
                ..Default::default()
            };
            let client = Client::new(options);
            let topic = client.topic("/");
            let subscribe_options = SubscriptionOptions {
                periodic: Some(Duration::from_millis(20)),
                all: Some(false),
                topics_only: Some(false),
                prefix: Some(true),
                extra: None,
            };

            let mut subscriber = match topic.subscribe(subscribe_options).await {
                Ok(subscriber) => subscriber,
                Err(_) => {
                    set_nt4_diagnostics(
                        &state,
                        false,
                        topic_bindings.len(),
                        "subscribe failed",
                    );
                    tokio::time::sleep(Duration::from_millis(250)).await;
                    continue;
                }
            };

            set_nt4_diagnostics(&state, true, topic_bindings.len(), "");
            let mut connect = Box::pin(client.connect());
            let mut last_topic_sync = std::time::Instant::now();
            loop {
                if !running.load(Ordering::SeqCst) {
                    set_nt4_diagnostics(&state, false, topic_bindings.len(), "stopped");
                    return;
                }

                tokio::select! {
                    connect_result = &mut connect => {
                        let _ = connect_result;
                        set_nt4_diagnostics(
                            &state,
                            false,
                            topic_bindings.len(),
                            "disconnected; retrying",
                        );
                        break;
                    }
                    recv_result = subscriber.recv() => {
                        let message = match recv_result {
                            Ok(message) => message,
                            Err(_) => {
                                set_nt4_diagnostics(
                                    &state,
                                    false,
                                    topic_bindings.len(),
                                    "receive failed; reconnecting",
                                );
                                break;
                            }
                        };
                        process_received_message(
                            message,
                            &state,
                            &mut topic_bindings,
                            &mut last_values,
                            &mut next_signal_id,
                        );
                        set_nt4_diagnostics(&state, true, topic_bindings.len(), "");
                    }
                    _ = tokio::time::sleep(Duration::from_millis(100)) => {
                        if last_topic_sync.elapsed() >= Duration::from_millis(500) {
                            sync_subscriber_topics(
                                &subscriber,
                                &state,
                                &mut topic_bindings,
                                &mut last_values,
                                &mut next_signal_id,
                            )
                            .await;
                            set_nt4_diagnostics(&state, true, topic_bindings.len(), "");
                            last_topic_sync = std::time::Instant::now();
                        }
                    }
                }
            }

            if running.load(Ordering::SeqCst) {
                tokio::time::sleep(Duration::from_millis(300)).await;
            }
        }
    });
}

fn init_nt4_diagnostics(state: &Arc<Mutex<DashboardState>>) {
    if let Ok(mut guard) = state.lock() {
        guard.upsert_descriptor(build_manifest_item(
            NT4_DIAG_CONNECTED_ID,
            SignalType::Bool,
            "NT4/__client/connected".to_string(),
        ));
        guard.upsert_descriptor(build_manifest_item(
            NT4_DIAG_TOPIC_COUNT_ID,
            SignalType::I64,
            "NT4/__client/topic_count".to_string(),
        ));
        guard.upsert_descriptor(build_manifest_item(
            NT4_DIAG_LAST_ERROR_ID,
            SignalType::Str,
            "NT4/__client/last_error".to_string(),
        ));
        guard.apply_update(NT4_DIAG_CONNECTED_ID, SignalValue::Bool(false));
        guard.apply_update(NT4_DIAG_TOPIC_COUNT_ID, SignalValue::I64(0));
        guard.apply_update(
            NT4_DIAG_LAST_ERROR_ID,
            SignalValue::Str("initializing".to_string()),
        );
    }
}

fn set_nt4_diagnostics(
    state: &Arc<Mutex<DashboardState>>,
    connected: bool,
    topic_count: usize,
    last_error: &str,
) {
    if let Ok(mut guard) = state.lock() {
        guard.apply_update(NT4_DIAG_CONNECTED_ID, SignalValue::Bool(connected));
        guard.apply_update(
            NT4_DIAG_TOPIC_COUNT_ID,
            SignalValue::I64(topic_count as i64),
        );
        guard.apply_update(
            NT4_DIAG_LAST_ERROR_ID,
            SignalValue::Str(last_error.to_string()),
        );
    }
}

async fn sync_subscriber_topics(
    subscriber: &nt_client::subscribe::Subscriber,
    state: &Arc<Mutex<DashboardState>>,
    topic_bindings: &mut HashMap<String, Nt4Binding>,
    last_values: &mut HashMap<u16, SignalValue>,
    next_signal_id: &mut u16,
) {
    let announced = subscriber.topics().await;
    for topic in announced.values() {
        let Some(binding) = ensure_binding(
            topic.name(),
            *topic.r#type(),
            state,
            topic_bindings,
            next_signal_id,
        ) else {
            continue;
        };
        apply_cached_topic_value(topic, binding, state, last_values);
    }
}

fn resolve_host_ipv4(host: &str) -> Option<std::net::Ipv4Addr> {
    let addrs = (host, 0).to_socket_addrs().ok()?;
    for addr in addrs {
        if let IpAddr::V4(v4) = addr.ip() {
            return Some(v4);
        }
    }
    None
}

fn process_received_message(
    message: ReceivedMessage,
    state: &Arc<Mutex<DashboardState>>,
    topic_bindings: &mut HashMap<String, Nt4Binding>,
    last_values: &mut HashMap<u16, SignalValue>,
    next_signal_id: &mut u16,
) {
    match message {
        ReceivedMessage::Announced(topic) => {
            let binding = ensure_binding(
                topic.name(),
                *topic.r#type(),
                state,
                topic_bindings,
                next_signal_id,
            );
            if let Some(binding) = binding {
                apply_cached_topic_value(&topic, binding, state, last_values);
            }
        }
        ReceivedMessage::Updated((topic, value)) => {
            let Some(binding) = ensure_binding(
                topic.name(),
                *topic.r#type(),
                state,
                topic_bindings,
                next_signal_id,
            ) else {
                return;
            };

            let Some(signal_value) = convert_value(*topic.r#type(), &value) else {
                return;
            };
            if signal_value.signal_type() != binding.signal_type {
                return;
            }
            if last_values
                .get(&binding.signal_id)
                .is_some_and(|previous| previous == &signal_value)
            {
                return;
            }

            if let Ok(mut guard) = state.lock() {
                guard.apply_update(binding.signal_id, signal_value.clone());
            }
            last_values.insert(binding.signal_id, signal_value);
        }
        ReceivedMessage::UpdateProperties(_) => {}
        ReceivedMessage::Unannounced { .. } => {}
    }
}

fn apply_cached_topic_value(
    topic: &AnnouncedTopic,
    binding: Nt4Binding,
    state: &Arc<Mutex<DashboardState>>,
    last_values: &mut HashMap<u16, SignalValue>,
) {
    let Some(value) = topic.value() else {
        return;
    };
    let Some(signal_value) = convert_value(*topic.r#type(), value) else {
        return;
    };
    if signal_value.signal_type() != binding.signal_type {
        return;
    }
    if last_values
        .get(&binding.signal_id)
        .is_some_and(|previous| previous == &signal_value)
    {
        return;
    }
    if let Ok(mut guard) = state.lock() {
        guard.apply_update(binding.signal_id, signal_value.clone());
    }
    last_values.insert(binding.signal_id, signal_value);
}

fn ensure_binding(
    nt_topic_name: &str,
    nt_data_type: DataType,
    state: &Arc<Mutex<DashboardState>>,
    topic_bindings: &mut HashMap<String, Nt4Binding>,
    next_signal_id: &mut u16,
) -> Option<Nt4Binding> {
    let signal_type = map_signal_type(nt_data_type)?;
    let path = to_arcp_nt4_path(nt_topic_name)?;

    if let Some(existing) = topic_bindings.get(nt_topic_name).copied() {
        if existing.signal_type == signal_type {
            if let Ok(mut guard) = state.lock() {
                if guard.descriptor_for(existing.signal_id).is_none() {
                    guard.upsert_descriptor(build_manifest_item(
                        existing.signal_id,
                        signal_type,
                        path,
                    ));
                }
            }
            return Some(existing);
        }
        topic_bindings.remove(nt_topic_name);
    }

    if topic_bindings.len() >= NT4_MAX_MIRRORED_TOPICS {
        return None;
    }

    let mut guard = state.lock().ok()?;
    let signal_id = guard
        .descriptors()
        .iter()
        .find(|descriptor| descriptor.path == path && descriptor.signal_type == signal_type)
        .map(|descriptor| descriptor.signal_id)
        .or_else(|| allocate_signal_id(&guard, next_signal_id))?;

    guard.upsert_descriptor(build_manifest_item(signal_id, signal_type, path));
    drop(guard);

    let binding = Nt4Binding {
        signal_id,
        signal_type,
    };
    topic_bindings.insert(nt_topic_name.to_string(), binding);
    Some(binding)
}

fn allocate_signal_id(state: &DashboardState, next_signal_id: &mut u16) -> Option<u16> {
    while *next_signal_id >= NT4_SIGNAL_ID_MIN {
        let candidate = *next_signal_id;
        *next_signal_id = next_signal_id.saturating_sub(1);
        if state.descriptor_for(candidate).is_none() {
            return Some(candidate);
        }
        if candidate == NT4_SIGNAL_ID_MIN {
            break;
        }
    }
    None
}

fn build_manifest_item(signal_id: u16, signal_type: SignalType, path: String) -> ManifestItem {
    ManifestItem {
        signal_id,
        signal_type,
        kind: SignalKind::Telemetry,
        access: SignalAccess::Observe,
        policy: SignalPolicy::OnChange,
        durability: SignalDurability::Volatile,
        persistence_scope: PersistenceScope::None,
        metadata_version: 1,
        metadata_hash: 0,
        path,
    }
}

fn to_arcp_nt4_path(nt_topic_path: &str) -> Option<String> {
    let mut normalized = nt_topic_path.trim();
    while normalized.starts_with('/') {
        normalized = &normalized[1..];
    }
    if normalized.is_empty() {
        return None;
    }

    let lower = normalized.to_ascii_lowercase();
    if lower.contains("/networktableconfig/") || lower.ends_with("/networktableconfig") {
        return None;
    }

    if normalized.starts_with("Athena/") {
        normalized = &normalized["Athena/".len()..];
        if normalized.is_empty() {
            return None;
        }
        return Some(format!("Athena/NT4/{normalized}"));
    }
    Some(format!("NT4/{normalized}"))
}

fn map_signal_type(nt_data_type: DataType) -> Option<SignalType> {
    match nt_data_type {
        DataType::Boolean => Some(SignalType::Bool),
        DataType::Double | DataType::Float => Some(SignalType::F64),
        DataType::Int => Some(SignalType::I64),
        DataType::String | DataType::Json => Some(SignalType::Str),
        DataType::BooleanArray => Some(SignalType::BoolArray),
        DataType::DoubleArray | DataType::FloatArray => Some(SignalType::F64Array),
        DataType::IntArray => Some(SignalType::I64Array),
        DataType::StringArray => Some(SignalType::StrArray),
        DataType::Raw | DataType::Rpc | DataType::Msgpack | DataType::Protobuf => {
            Some(SignalType::Str)
        }
    }
}

fn convert_value(nt_data_type: DataType, value: &rmpv::Value) -> Option<SignalValue> {
    match nt_data_type {
        DataType::Boolean => bool::from_value(value).map(SignalValue::Bool),
        DataType::Double => f64::from_value(value).map(SignalValue::F64),
        DataType::Float => f32::from_value(value).map(|value| SignalValue::F64(value as f64)),
        DataType::Int => i64::from_value(value).map(SignalValue::I64),
        DataType::String | DataType::Json => String::from_value(value).map(SignalValue::Str),
        DataType::BooleanArray => Vec::<bool>::from_value(value).map(SignalValue::BoolArray),
        DataType::DoubleArray => Vec::<f64>::from_value(value).map(SignalValue::F64Array),
        DataType::FloatArray => Vec::<f32>::from_value(value)
            .map(|values| SignalValue::F64Array(values.into_iter().map(f64::from).collect())),
        DataType::IntArray => Vec::<i64>::from_value(value).map(SignalValue::I64Array),
        DataType::StringArray => Vec::<String>::from_value(value).map(SignalValue::StrArray),
        DataType::Raw | DataType::Rpc | DataType::Msgpack | DataType::Protobuf => {
            Some(SignalValue::Str(format!("{value:?}")))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::to_arcp_nt4_path;

    #[test]
    fn maps_athena_topics_to_legacy_nt4_namespace() {
        let mapped = to_arcp_nt4_path("/Athena/Drivetrain/Speed");
        assert_eq!(mapped.as_deref(), Some("Athena/NT4/Drivetrain/Speed"));
    }

    #[test]
    fn maps_non_athena_topics_to_nt4_namespace() {
        let mapped = to_arcp_nt4_path("/SmartDashboard/LoopTimeMs");
        assert_eq!(mapped.as_deref(), Some("NT4/SmartDashboard/LoopTimeMs"));
    }

    #[test]
    fn excludes_network_table_config_topics() {
        assert_eq!(
            to_arcp_nt4_path("/.schema/NetworkTableConfig"),
            None
        );
    }

    #[test]
    fn maps_non_primitive_nt_types_as_strings() {
        use super::{convert_value, map_signal_type};
        use nt_client::data::r#type::DataType;

        assert_eq!(map_signal_type(DataType::Raw), Some(arcp_core::SignalType::Str));
        assert_eq!(
            map_signal_type(DataType::Protobuf),
            Some(arcp_core::SignalType::Str)
        );
        let value = rmpv::Value::Binary(vec![1, 2, 3, 4].into());
        let converted = convert_value(DataType::Raw, &value);
        assert!(matches!(converted, Some(arcp_core::SignalValue::Str(_))));
    }
}
