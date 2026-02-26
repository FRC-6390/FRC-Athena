use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;

use arcp_core::{
    SignalAccess, SignalDescriptor, SignalDurability, SignalPolicy, SignalType, SignalValue,
};
use nt_client::data::r#type::{DataType, NetworkTableData};
use nt_client::data::SubscriptionOptions;
use nt_client::subscribe::ReceivedMessage;
use nt_client::{Client, NTAddr, NewClientOptions};

use crate::realtime::PublishMessage;

#[derive(Clone, Copy, Debug)]
struct Nt4Binding {
    signal_id: u16,
    signal_type: SignalType,
}

pub(crate) fn run_nt4_bridge_loop(
    running: Arc<AtomicBool>,
    descriptors: Arc<Mutex<HashMap<u16, SignalDescriptor>>>,
    publish_tx: mpsc::Sender<PublishMessage>,
    max_signals: u16,
    unsecure_port: u16,
) {
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

        while running.load(Ordering::SeqCst) {
            let options = NewClientOptions {
                addr: NTAddr::Local,
                unsecure_port,
                secure_port: None,
                name: "arcp-nt4-bridge".to_string(),
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
                    tokio::time::sleep(Duration::from_millis(250)).await;
                    continue;
                }
            };

            let mut connect = Box::pin(client.connect());
            loop {
                if !running.load(Ordering::SeqCst) {
                    return;
                }

                tokio::select! {
                    connect_result = &mut connect => {
                        let _ = connect_result;
                        break;
                    }
                    recv_result = subscriber.recv() => {
                        let message = match recv_result {
                            Ok(message) => message,
                            Err(_) => break,
                        };
                        process_received_message(
                            message,
                            &descriptors,
                            &publish_tx,
                            max_signals,
                            &mut topic_bindings,
                            &mut last_values,
                        );
                    }
                    _ = tokio::time::sleep(Duration::from_millis(100)) => {}
                }
            }

            if running.load(Ordering::SeqCst) {
                tokio::time::sleep(Duration::from_millis(300)).await;
            }
        }
    });
}

fn process_received_message(
    message: ReceivedMessage,
    descriptors: &Arc<Mutex<HashMap<u16, SignalDescriptor>>>,
    publish_tx: &mpsc::Sender<PublishMessage>,
    max_signals: u16,
    topic_bindings: &mut HashMap<String, Nt4Binding>,
    last_values: &mut HashMap<u16, SignalValue>,
) {
    match message {
        ReceivedMessage::Announced(topic) => {
            let _ = ensure_binding(
                topic.name(),
                *topic.r#type(),
                descriptors,
                max_signals,
                topic_bindings,
            );
        }
        ReceivedMessage::Updated((topic, value)) => {
            let Some(binding) = ensure_binding(
                topic.name(),
                *topic.r#type(),
                descriptors,
                max_signals,
                topic_bindings,
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

            last_values.insert(binding.signal_id, signal_value.clone());
            let _ = publish_tx.send(PublishMessage::Value {
                signal_id: binding.signal_id,
                value: signal_value,
            });
        }
        ReceivedMessage::UpdateProperties(_) => {}
        ReceivedMessage::Unannounced { .. } => {}
    }
}

fn ensure_binding(
    nt_topic_name: &str,
    nt_data_type: DataType,
    descriptors: &Arc<Mutex<HashMap<u16, SignalDescriptor>>>,
    max_signals: u16,
    topic_bindings: &mut HashMap<String, Nt4Binding>,
) -> Option<Nt4Binding> {
    if let Some(existing) = topic_bindings.get(nt_topic_name).copied() {
        return Some(existing);
    }

    let signal_type = map_signal_type(nt_data_type)?;
    let path = to_arcp_nt4_path(nt_topic_name)?;
    let mut descriptor_map = descriptors.lock().ok()?;

    let signal_id = descriptor_map
        .values()
        .find(|descriptor| descriptor.path == path && descriptor.signal_type == signal_type)
        .map(|descriptor| descriptor.signal_id)
        .or_else(|| {
            (1..=max_signals)
                .rev()
                .find(|candidate| !descriptor_map.contains_key(candidate))
        })?;

    descriptor_map.entry(signal_id).or_insert_with(|| {
        let mut descriptor = SignalDescriptor::telemetry(
            signal_id,
            signal_type,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Volatile,
            path,
        );
        descriptor.metadata_hash = descriptor.metadata_fingerprint();
        descriptor
    });

    let binding = Nt4Binding {
        signal_id,
        signal_type,
    };
    topic_bindings.insert(nt_topic_name.to_string(), binding);
    Some(binding)
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
    }
    if normalized.is_empty() {
        return None;
    }

    Some(format!("Athena/NT4/{normalized}"))
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
        DataType::Raw | DataType::Rpc | DataType::Msgpack | DataType::Protobuf => None,
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
        DataType::Raw | DataType::Rpc | DataType::Msgpack | DataType::Protobuf => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_mapping_keeps_nt4_root_and_filters_network_table_config() {
        assert_eq!(
            to_arcp_nt4_path("/Athena/Drivetrain/Speed"),
            Some("Athena/NT4/Drivetrain/Speed".to_string())
        );
        assert_eq!(
            to_arcp_nt4_path("/Robot/Status"),
            Some("Athena/NT4/Robot/Status".to_string())
        );
        assert_eq!(to_arcp_nt4_path("/Athena/NetworkTableConfig/Details"), None);
        assert_eq!(
            to_arcp_nt4_path("/Athena/Mechanism/NetworkTableConfig"),
            None
        );
    }

    #[test]
    fn type_mapping_ignores_unsupported_nt_payload_types() {
        assert_eq!(map_signal_type(DataType::Boolean), Some(SignalType::Bool));
        assert_eq!(
            map_signal_type(DataType::IntArray),
            Some(SignalType::I64Array)
        );
        assert_eq!(map_signal_type(DataType::Protobuf), None);
        assert_eq!(map_signal_type(DataType::Raw), None);
    }
}
