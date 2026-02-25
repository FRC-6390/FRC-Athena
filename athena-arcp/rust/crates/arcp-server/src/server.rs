use std::collections::HashMap;
use std::net::{SocketAddr, TcpListener, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use arcp_core::{encode_event_into, RuntimeEvent, SignalDescriptor, SignalValue};

use crate::config::ArcpServerConfig;
use crate::control::run_control_loop;
use crate::realtime::{run_realtime_loop, PublishMessage};

pub struct ArcpServer {
    running: Arc<AtomicBool>,
    config: ArcpServerConfig,
    subscribers: Arc<Mutex<Vec<SocketAddr>>>,
    descriptors: Arc<Mutex<HashMap<u16, SignalDescriptor>>>,
    layout_store: Arc<Mutex<HashMap<String, String>>>,
    publish_tx: Mutex<Option<mpsc::Sender<PublishMessage>>>,
    event_tx: Mutex<Option<mpsc::Sender<RuntimeEvent>>>,
    event_rx: Mutex<Option<mpsc::Receiver<RuntimeEvent>>>,
    control_thread: Mutex<Option<JoinHandle<()>>>,
    realtime_thread: Mutex<Option<JoinHandle<()>>>,
    bound_control_port: Arc<Mutex<u16>>,
    bound_realtime_port: Arc<Mutex<u16>>,
}

impl ArcpServer {
    pub fn new(config: ArcpServerConfig) -> Self {
        Self {
            running: Arc::new(AtomicBool::new(false)),
            config,
            subscribers: Arc::new(Mutex::new(Vec::new())),
            descriptors: Arc::new(Mutex::new(HashMap::new())),
            layout_store: Arc::new(Mutex::new(HashMap::new())),
            publish_tx: Mutex::new(None),
            event_tx: Mutex::new(None),
            event_rx: Mutex::new(None),
            control_thread: Mutex::new(None),
            realtime_thread: Mutex::new(None),
            bound_control_port: Arc::new(Mutex::new(0)),
            bound_realtime_port: Arc::new(Mutex::new(0)),
        }
    }

    pub fn start(&self) -> Result<(), &'static str> {
        if self.running.swap(true, Ordering::SeqCst) {
            return Err("server already running");
        }

        let control_listener = TcpListener::bind(("0.0.0.0", self.config.control_port))
            .map_err(|_| "failed to bind control listener")?;
        control_listener
            .set_nonblocking(true)
            .map_err(|_| "failed to configure control listener")?;
        let control_port = control_listener
            .local_addr()
            .map_err(|_| "failed to read bound control port")?
            .port();

        let realtime_socket = UdpSocket::bind(("0.0.0.0", self.config.realtime_port))
            .map_err(|_| "failed to bind realtime socket")?;
        realtime_socket
            .set_write_timeout(Some(Duration::from_millis(50)))
            .map_err(|_| "failed to configure realtime socket")?;
        let realtime_port = realtime_socket
            .local_addr()
            .map_err(|_| "failed to read bound realtime port")?
            .port();

        if let Ok(mut port) = self.bound_control_port.lock() {
            *port = control_port;
        }
        if let Ok(mut port) = self.bound_realtime_port.lock() {
            *port = realtime_port;
        }

        let (publish_tx, publish_rx) = mpsc::channel::<PublishMessage>();
        let (event_tx, event_rx) = mpsc::channel::<RuntimeEvent>();
        if let Ok(mut tx) = self.publish_tx.lock() {
            *tx = Some(publish_tx);
        }
        if let Ok(mut tx) = self.event_tx.lock() {
            *tx = Some(event_tx.clone());
        }
        if let Ok(mut rx) = self.event_rx.lock() {
            *rx = Some(event_rx);
        }

        let running_for_control = Arc::clone(&self.running);
        let subscribers_for_control = Arc::clone(&self.subscribers);
        let descriptors_for_control = Arc::clone(&self.descriptors);
        let layout_store_for_control = Arc::clone(&self.layout_store);
        let control_handle = thread::spawn(move || {
            run_control_loop(
                control_listener,
                subscribers_for_control,
                descriptors_for_control,
                layout_store_for_control,
                event_tx,
                running_for_control,
            );
        });

        let running_for_realtime = Arc::clone(&self.running);
        let subscribers_for_realtime = Arc::clone(&self.subscribers);
        let realtime_handle = thread::spawn(move || {
            run_realtime_loop(
                realtime_socket,
                publish_rx,
                subscribers_for_realtime,
                running_for_realtime,
            );
        });

        if let Ok(mut handle) = self.control_thread.lock() {
            *handle = Some(control_handle);
        }
        if let Ok(mut handle) = self.realtime_thread.lock() {
            *handle = Some(realtime_handle);
        }

        Ok(())
    }

    pub fn stop(&self) {
        if !self.running.swap(false, Ordering::SeqCst) {
            return;
        }

        if let Ok(mut tx) = self.publish_tx.lock() {
            tx.take();
        }
        if let Ok(mut tx) = self.event_tx.lock() {
            tx.take();
        }

        if let Ok(mut handle) = self.control_thread.lock() {
            if let Some(join_handle) = handle.take() {
                let _ = join_handle.join();
            }
        }
        if let Ok(mut handle) = self.realtime_thread.lock() {
            if let Some(join_handle) = handle.take() {
                let _ = join_handle.join();
            }
        }

        if let Ok(mut subscribers) = self.subscribers.lock() {
            subscribers.clear();
        }
    }

    pub fn register_signal(&self, descriptor: SignalDescriptor) -> Result<(), &'static str> {
        let mut descriptor = descriptor;
        if descriptor.metadata_version == 0 {
            descriptor.metadata_version = 1;
        }
        if descriptor.metadata_hash == 0 {
            descriptor.metadata_hash = descriptor.metadata_fingerprint();
        }
        descriptor.validate()?;
        if descriptor.signal_id == 0 {
            return Err("signal_id 0 is reserved");
        }
        if descriptor.signal_id > self.config.max_signals {
            return Err("signal_id exceeds configured max_signals");
        }
        match self.descriptors.lock() {
            Ok(mut map) => {
                map.insert(descriptor.signal_id, descriptor);
                Ok(())
            }
            Err(_) => Err("descriptor registry unavailable"),
        }
    }

    pub fn publish_bool(&self, signal_id: u16, value: bool) -> Result<(), &'static str> {
        self.publish_value(signal_id, SignalValue::Bool(value))
    }

    pub fn publish_i64(&self, signal_id: u16, value: i64) -> Result<(), &'static str> {
        self.publish_value(signal_id, SignalValue::I64(value))
    }

    pub fn publish_f64(&self, signal_id: u16, value: f64) -> Result<(), &'static str> {
        self.publish_value(signal_id, SignalValue::F64(value))
    }

    pub fn publish_string(&self, signal_id: u16, value: String) -> Result<(), &'static str> {
        self.publish_value(signal_id, SignalValue::Str(value))
    }

    pub fn publish_bool_array(&self, signal_id: u16, value: Vec<bool>) -> Result<(), &'static str> {
        self.publish_value(signal_id, SignalValue::BoolArray(value))
    }

    pub fn publish_i64_array(&self, signal_id: u16, value: Vec<i64>) -> Result<(), &'static str> {
        self.publish_value(signal_id, SignalValue::I64Array(value))
    }

    pub fn publish_f64_array(&self, signal_id: u16, value: Vec<f64>) -> Result<(), &'static str> {
        self.publish_value(signal_id, SignalValue::F64Array(value))
    }

    pub fn publish_string_array(
        &self,
        signal_id: u16,
        value: Vec<String>,
    ) -> Result<(), &'static str> {
        self.publish_value(signal_id, SignalValue::StrArray(value))
    }

    pub fn poll_events_into(&self, out: &mut [u8]) -> usize {
        let receiver_guard = match self.event_rx.lock() {
            Ok(guard) => guard,
            Err(_) => return 0,
        };
        let receiver = match receiver_guard.as_ref() {
            Some(receiver) => receiver,
            None => return 0,
        };

        let mut written = 0_usize;
        let mut encoded = Vec::with_capacity(64);
        loop {
            let event = match receiver.try_recv() {
                Ok(event) => event,
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => break,
            };
            if encode_event_into(&event, &mut encoded).is_err() {
                continue;
            }
            let record_len = encoded.len();
            if record_len > u16::MAX as usize {
                continue;
            }
            let needed = 2 + record_len;
            if written + needed > out.len() {
                break;
            }
            let len_bytes = (record_len as u16).to_le_bytes();
            out[written..written + 2].copy_from_slice(&len_bytes);
            written += 2;
            out[written..written + record_len].copy_from_slice(&encoded);
            written += record_len;
        }
        written
    }

    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::SeqCst)
    }

    pub fn config(&self) -> &ArcpServerConfig {
        &self.config
    }

    pub fn control_port(&self) -> u16 {
        match self.bound_control_port.lock() {
            Ok(port) if *port != 0 => *port,
            _ => self.config.control_port,
        }
    }

    pub fn realtime_port(&self) -> u16 {
        match self.bound_realtime_port.lock() {
            Ok(port) if *port != 0 => *port,
            _ => self.config.realtime_port,
        }
    }

    pub fn store_layout(
        &self,
        name: impl Into<String>,
        layout_json: impl Into<String>,
    ) -> Result<(), &'static str> {
        let name = name.into();
        if !is_valid_layout_name(&name) {
            return Err("invalid layout name");
        }

        let layout_json = layout_json.into();
        if layout_json.is_empty() {
            return Err("layout payload is empty");
        }
        if layout_json.len() > MAX_LAYOUT_BYTES {
            return Err("layout payload exceeds max size");
        }

        let mut store = self
            .layout_store
            .lock()
            .map_err(|_| "layout store unavailable")?;
        if !store.contains_key(&name) && store.len() >= MAX_LAYOUTS {
            return Err("layout store is full");
        }
        store.insert(name, layout_json);
        Ok(())
    }

    pub fn load_layout(&self, name: &str) -> Option<String> {
        let store = self.layout_store.lock().ok()?;
        store.get(name).cloned()
    }

    pub fn list_layout_names(&self) -> Result<Vec<String>, &'static str> {
        let mut names = self
            .layout_store
            .lock()
            .map_err(|_| "layout store unavailable")?
            .keys()
            .cloned()
            .collect::<Vec<_>>();
        names.sort();
        Ok(names)
    }

    fn publish_value(&self, signal_id: u16, value: SignalValue) -> Result<(), &'static str> {
        if !self.is_running() {
            return Err("server not running");
        }
        self.validate_value_type(signal_id, &value)?;
        let sender = match self.publish_tx.lock() {
            Ok(guard) => guard.clone(),
            Err(_) => return Err("publisher mutex poisoned"),
        };
        let tx = sender.ok_or("publish channel unavailable")?;
        tx.send(PublishMessage::Value { signal_id, value })
            .map_err(|_| "failed to enqueue telemetry update")
    }

    fn validate_value_type(&self, signal_id: u16, value: &SignalValue) -> Result<(), &'static str> {
        let descriptors = self
            .descriptors
            .lock()
            .map_err(|_| "descriptor registry unavailable")?;
        let descriptor = descriptors
            .get(&signal_id)
            .ok_or("signal not registered before publish")?;
        if descriptor.signal_type != value.signal_type() {
            return Err("published value type does not match signal descriptor");
        }
        Ok(())
    }
}

const MAX_LAYOUT_BYTES: usize = 512 * 1024;
const MAX_LAYOUTS: usize = 64;

fn is_valid_layout_name(name: &str) -> bool {
    if name.is_empty() || name.len() > 64 {
        return false;
    }
    name.bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.'))
}

impl Drop for ArcpServer {
    fn drop(&mut self) {
        self.stop();
    }
}
