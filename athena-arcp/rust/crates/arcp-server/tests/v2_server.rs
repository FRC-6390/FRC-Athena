//! End-to-end tests for the v2 server over real TCP and UDP sockets on localhost.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream, UdpSocket};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use arcp_core::v2::control::{
    decode_server, encode_client, frame, take_frame, ClientMessage, Rate, ReplyData, ServerMessage, Status, WriteReply,
};
use arcp_core::v2::record::{decode_stream, DictEntry, Dictionary, Marker, Record, StructSchema, DICT_FLAG_WRITABLE};
use arcp_core::v2::{Entry, Value};
use arcp_server::v2::{ArcpServerV2, RobotHandler, ServerConfig};

#[derive(Default)]
struct RecordingHandler {
    writes: Mutex<Vec<(u64, Value)>>,
    actions: Mutex<Vec<(u64, bool)>>,
}

impl RobotHandler for RecordingHandler {
    fn write(&self, signal_id: u64, value: Value) -> WriteReply {
        self.writes.lock().unwrap().push((signal_id, value.clone()));
        if signal_id == 1 {
            WriteReply::rejected("rejected: locked (InMatch)", Value::F64(0.0))
        } else {
            WriteReply::accepted(value)
        }
    }

    fn action(&self, signal_id: u64, run: bool) -> Result<(), String> {
        self.actions.lock().unwrap().push((signal_id, run));
        Ok(())
    }
}

const ANGLE: u64 = 1;
const POSE: u64 = 2;
const ENABLED: u64 = 3;
const OTHER: u64 = 4;

fn dictionary() -> Dictionary {
    let entry = |id: u64, path: &str, tag: u8, flags: u8, lock: &str, schema: u64| DictEntry {
        signal_id: id,
        path: path.into(),
        type_tag: tag,
        unit: String::new(),
        role: 1,
        flags,
        lock_group: lock.into(),
        struct_schema_id: schema,
    };
    Dictionary {
        entries: vec![
            entry(ANGLE, "robot/turret/angle", 3, DICT_FLAG_WRITABLE, "InMatch", 0),
            entry(POSE, "robot/turret/pose", 9, 0, "", 1),
            entry(ENABLED, "robot/turret/enabled", 1, 0, "", 0),
            entry(OTHER, "robot/intake/speed", 3, 0, "", 0),
        ],
    }
}

fn schemas() -> Vec<StructSchema> {
    vec![StructSchema { id: 1, type_name: "Pose2d".into(), schema: "Translation2d translation;Rotation2d rotation".into() }]
}

struct Client {
    tcp: TcpStream,
    buf: Vec<u8>,
    next_id: u32,
    pushes: Vec<ServerMessage>,
}

impl Client {
    fn connect(addr: SocketAddr) -> Client {
        let tcp = TcpStream::connect(addr).unwrap();
        tcp.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        let mut c = Client { tcp, buf: Vec::new(), next_id: 1, pushes: Vec::new() };
        // The server announces the schema on connect (request id 0).
        let (id, msg) = c.read_message();
        assert_eq!(id, 0);
        assert!(matches!(msg, ServerMessage::Schema { .. }));
        c
    }

    fn read_message(&mut self) -> (u32, ServerMessage) {
        loop {
            if let Some((consumed, body)) = take_frame(&self.buf).unwrap() {
                let decoded = decode_server(body).unwrap();
                self.buf.drain(..consumed);
                return decoded;
            }
            let mut chunk = [0u8; 4096];
            let n = self.tcp.read(&mut chunk).expect("tcp read");
            assert!(n > 0, "server closed connection");
            self.buf.extend_from_slice(&chunk[..n]);
        }
    }

    /// Sends a request and returns the reply, keeping any pushes received first.
    fn request(&mut self, msg: ClientMessage) -> ServerMessage {
        let id = self.next_id;
        self.next_id += 1;
        self.tcp.write_all(&frame(&encode_client(id, &msg)).unwrap()).unwrap();
        loop {
            let (rid, reply) = self.read_message();
            if rid == id {
                return reply;
            }
            self.pushes.push(reply);
        }
    }

    fn reply(&mut self, msg: ClientMessage) -> (Status, String, ReplyData) {
        match self.request(msg) {
            ServerMessage::Reply { status, reason, data } => (status, reason, data),
            other => panic!("expected reply, got {other:?}"),
        }
    }

    fn hello(&mut self, udp_port: u16) {
        let (status, _, data) = self.reply(ClientMessage::Hello {
            client_name: "test".into(),
            protocol_version: 2,
            udp_port,
        });
        assert_eq!(status, Status::Ok);
        assert!(matches!(data, ReplyData::Session { .. }));
    }

    /// Writes a value and returns the robot's `{accepted, reason, actualValue}` reply.
    fn write(&mut self, signal_id: u64, value: Value) -> WriteReply {
        let id = self.next_id;
        self.next_id += 1;
        self.tcp.write_all(&frame(&encode_client(id, &ClientMessage::Write { signal_id, value })).unwrap()).unwrap();
        loop {
            let (rid, msg) = self.read_message();
            if rid == id {
                match msg {
                    ServerMessage::WriteReply(reply) => return reply,
                    other => panic!("expected write reply, got {other:?}"),
                }
            }
            self.pushes.push(msg);
        }
    }

    fn subscribe(&mut self, pattern: &str, rate: Rate) -> u32 {
        let (status, reason, data) = self.reply(ClientMessage::Subscribe { pattern: pattern.into(), rate });
        assert_eq!(status, Status::Ok, "{reason}");
        match data {
            ReplyData::Subscription { subscription_id } => subscription_id,
            other => panic!("{other:?}"),
        }
    }
}

fn udp_pair() -> (UdpSocket, u16) {
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock.set_read_timeout(Some(Duration::from_millis(1500))).unwrap();
    let port = sock.local_addr().unwrap().port();
    (sock, port)
}

/// Collects every datagram that arrives within the timeout as decoded records.
fn drain_udp(sock: &UdpSocket) -> Vec<Record> {
    let mut out = Vec::new();
    let mut buf = [0u8; 65536];
    while let Ok((n, _)) = sock.recv_from(&mut buf) {
        out.extend(decode_stream(&buf[..n]).expect("valid datagram"));
    }
    out
}

fn frames(records: &[Record]) -> Vec<&arcp_core::v2::record::Frame> {
    records
        .iter()
        .filter_map(|r| match r {
            Record::Frame(f) => Some(f),
            _ => None,
        })
        .collect()
}

fn server_with(config: ServerConfig) -> (ArcpServerV2, Arc<RecordingHandler>) {
    let handler = Arc::new(RecordingHandler::default());
    let server = ArcpServerV2::start(config, handler.clone()).unwrap();
    server.publish_dictionary(dictionary(), schemas());
    (server, handler)
}

fn server() -> (ArcpServerV2, Arc<RecordingHandler>) {
    server_with(ServerConfig::default())
}

fn angle(v: f64) -> Entry {
    Entry { signal_id: ANGLE, value: Value::F64(v) }
}

#[test]
fn dictionary_is_announced_on_connect_and_on_change() {
    let (server, _) = server();
    let mut client = Client::connect(server.control_addr());

    // A change is pushed to connected clients with request id 0.
    let mut extended = dictionary();
    extended.entries.push(DictEntry {
        signal_id: 50,
        path: "robot/new/signal".into(),
        type_tag: 2,
        unit: "ticks".into(),
        role: 2,
        flags: 0,
        lock_group: String::new(),
        struct_schema_id: 0,
    });
    server.publish_dictionary(extended.clone(), schemas());
    let (id, msg) = client.read_message();
    assert_eq!(id, 0);
    match msg {
        ServerMessage::Schema { dictionary: dict, .. } => assert_eq!(dict, extended),
        other => panic!("{other:?}"),
    }
}

#[test]
fn schema_request_returns_full_dictionary_with_metadata() {
    let (server, _) = server();
    let mut client = Client::connect(server.control_addr());
    match client.request(ClientMessage::SchemaRequest) {
        ServerMessage::Schema { dictionary: dict, schemas: s } => {
            assert_eq!(dict, dictionary());
            assert_eq!(s, schemas());
            let angle = &dict.entries[0];
            assert!(angle.writable());
            assert_eq!(angle.lock_group, "InMatch");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn subscribe_sends_snapshot_then_only_matching_signals_per_cycle() {
    let (server, _) = server();
    let (udp, port) = udp_pair();
    let mut client = Client::connect(server.control_addr());
    client.hello(port);

    server.publish_cycle(1_000_000, &[angle(10.0), Entry { signal_id: OTHER, value: Value::F64(5.0) }]);
    let _ = drain_udp(&udp); // nothing subscribed yet

    client.subscribe("robot/turret/**", Rate::OnChange);
    let snapshot = drain_udp(&udp);
    let snap = frames(&snapshot);
    assert_eq!(snap.len(), 1, "one snapshot frame");
    assert_eq!(snap[0].entries.len(), 1, "only the matching signal is snapshotted");
    assert_eq!(snap[0].entries[0].signal_id, ANGLE);

    // Unchanged value: nothing for OnChange. Other subsystem: never sent to this subscriber.
    server.publish_cycle(1_020_000, &[angle(10.0), Entry { signal_id: OTHER, value: Value::F64(6.0) }]);
    assert!(frames(&drain_udp(&udp)).is_empty());

    // Changed value: one batched frame containing the changed matching signal.
    server.publish_cycle(1_040_000, &[angle(11.5), Entry { signal_id: OTHER, value: Value::F64(7.0) }]);
    let batch = drain_udp(&udp);
    let f = frames(&batch);
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].entries.len(), 1);
    assert_eq!(f[0].entries[0].value, Value::F64(11.5));
    assert_eq!(f[0].timestamp_micros, 1_040_000);
    assert!(!f[0].keyframe());
}

#[test]
fn hz_rate_paces_updates_and_keyframes_arrive_periodically() {
    let (server, _) = server();
    let (udp, port) = udp_pair();
    let mut client = Client::connect(server.control_addr());
    client.hello(port);
    client.subscribe("robot/turret/angle", Rate::Hz(10.0));

    // Cycles at 1 kHz for 0.5 s of robot time. At 10 Hz only about 5 updates are due.
    let mut sent = Vec::new();
    for i in 0..=500u64 {
        server.publish_cycle(i * 1000, &[angle(i as f64)]);
        sent.push(i);
    }
    let received = frames(&drain_udp(&udp)).len();
    assert!((4..=8).contains(&received), "expected about 5 frames at 10 Hz, got {received}");
    assert_eq!(sent.len(), 501);
}

#[test]
fn one_cycle_produces_one_batched_datagram() {
    let (server, _) = server();
    let (udp, port) = udp_pair();
    let mut client = Client::connect(server.control_addr());
    client.hello(port);
    client.subscribe("robot/**", Rate::OnChange);
    drain_udp(&udp);

    server.publish_cycle(5_000_000, &[angle(1.0), Entry { signal_id: ENABLED, value: Value::Bool(true) }]);
    let mut count = 0;
    let mut buf = [0u8; 65536];
    let mut total_entries = 0;
    while let Ok((n, _)) = udp.recv_from(&mut buf) {
        count += 1;
        for r in decode_stream(&buf[..n]).unwrap() {
            if let Record::Frame(f) = r {
                total_entries += f.entries.len();
            }
        }
    }
    assert_eq!(count, 1, "both changed signals fit in one datagram");
    assert_eq!(total_entries, 2 + 0);
}

#[test]
fn keyframe_contains_every_subscribed_signal() {
    let (server, _) = server_with(ServerConfig {
        keyframe_interval: Duration::from_millis(100),
        ..ServerConfig::default()
    });
    let (udp, port) = udp_pair();
    let mut client = Client::connect(server.control_addr());
    client.hello(port);
    client.subscribe("robot/turret/**", Rate::OnChange);
    drain_udp(&udp);
    server.publish_cycle(0, &[angle(1.0), Entry { signal_id: ENABLED, value: Value::Bool(true) }]);
    drain_udp(&udp);
    // 200 ms of robot time later the keyframe is due even though nothing changed.
    server.publish_cycle(200_000, &[angle(1.0), Entry { signal_id: ENABLED, value: Value::Bool(true) }]);
    let f = frames(&drain_udp(&udp)).into_iter().find(|f| f.keyframe()).map(|f| f.entries.len());
    assert_eq!(f, Some(2));
}

#[test]
fn large_cycles_are_fragmented_and_recoverable() {
    let (server, _) = server_with(ServerConfig { mtu: 256, ..ServerConfig::default() });
    let (udp, port) = udp_pair();
    let mut client = Client::connect(server.control_addr());
    client.hello(port);
    client.subscribe("robot/**", Rate::OnChange);
    drain_udp(&udp);

    let entries: Vec<Entry> = (0..40)
        .map(|i| Entry { signal_id: 100 + i, value: Value::F64(i as f64) })
        .collect();
    // Dictionary has no entries for ids 100+, so re-announce a larger dictionary.
    let mut dict = dictionary();
    for i in 0..40u64 {
        dict.entries.push(DictEntry {
            signal_id: 100 + i,
            path: format!("robot/bulk/{i}"),
            type_tag: 3,
            unit: String::new(),
            role: 2,
            flags: 0,
            lock_group: String::new(),
            struct_schema_id: 0,
        });
    }
    server.publish_dictionary(dict, schemas());
    drain_udp(&udp);
    server.publish_cycle(9_000_000, &entries);

    let records = drain_udp(&udp);
    let f = frames(&records);
    assert!(f.len() > 1, "expected fragments, got {}", f.len());
    assert!(f.iter().all(|x| x.fragmented() && x.fragment_count as usize == f.len()));
    let mut ids: Vec<u64> = f.iter().flat_map(|x| x.entries.iter().map(|e| e.signal_id)).collect();
    ids.sort_unstable();
    assert_eq!(ids, (100..140).collect::<Vec<u64>>());
}

#[test]
fn markers_reach_subscribers_unfiltered() {
    let (server, _) = server();
    let (udp, port) = udp_pair();
    let mut client = Client::connect(server.control_addr());
    client.hello(port);
    server.marker(Marker { timestamp_micros: 42, kind: arcp_core::v2::record::marker_kind::FAULT, message: "boom".into(), signal_id: 0 });
    let records = drain_udp(&udp);
    assert!(records.iter().any(|r| matches!(r, Record::Marker(m) if m.message == "boom")));
}

#[test]
fn writes_are_forwarded_and_robot_reason_is_returned() {
    let (server, handler) = server();
    let mut client = Client::connect(server.control_addr());
    client.hello(9);

    let reply = client.write(ANGLE, Value::F64(3.0));
    assert!(!reply.accepted);
    assert_eq!(reply.reason, "rejected: locked (InMatch)");
    assert_eq!(reply.actual_value, Value::F64(0.0));

    let reply = client.write(OTHER, Value::F64(3.0));
    assert!(reply.accepted);
    assert_eq!(reply.actual_value, Value::F64(3.0));
    assert_eq!(handler.writes.lock().unwrap().len(), 2, "both forwarded, none pre-blocked");

    let (status, _, _) = client.reply(ClientMessage::ActionRun { signal_id: 9 });
    assert_eq!(status, Status::Ok);
    let (status, _, _) = client.reply(ClientMessage::ActionCancel { signal_id: 9 });
    assert_eq!(status, Status::Ok);
    assert_eq!(*handler.actions.lock().unwrap(), vec![(9, true), (9, false)]);
}

#[test]
fn requests_before_hello_are_refused_except_time_sync() {
    let (server, _) = server();
    let mut client = Client::connect(server.control_addr());
    let (status, reason, _) = client.reply(ClientMessage::FileList);
    assert_eq!(status, Status::Error);
    assert_eq!(reason, "hello required");
    match client.request(ClientMessage::TimePing { client_micros: 77 }) {
        ServerMessage::TimePong { client_micros, server_micros } => {
            assert_eq!(client_micros, 77);
            assert!(server_micros > 0);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn subscribe_validates_pattern_and_rate() {
    let (server, _) = server();
    let (_udp, port) = udp_pair();
    let mut client = Client::connect(server.control_addr());
    client.hello(port);
    let (status, _, _) = client.reply(ClientMessage::Subscribe { pattern: "robot//x".into(), rate: Rate::OnChange });
    assert_eq!(status, Status::Error);
    let (status, _, _) = client.reply(ClientMessage::Subscribe { pattern: "robot/**".into(), rate: Rate::Hz(-1.0) });
    assert_eq!(status, Status::Error);
}

#[test]
fn file_service_lists_downloads_and_deletes_within_root() {
    let root: PathBuf = std::env::temp_dir().join(format!("arcp-v2-files-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    std::fs::write(root.join("match-1.athrp"), b"0123456789").unwrap();
    std::fs::write(root.join("tuning.json"), b"{}").unwrap();

    let handler = Arc::new(RecordingHandler::default());
    let server = ArcpServerV2::start(ServerConfig { file_root: Some(root.clone()), ..ServerConfig::default() }, handler).unwrap();
    let (_udp, port) = udp_pair();
    let mut client = Client::connect(server.control_addr());
    client.hello(port);

    match client.reply(ClientMessage::FileList) {
        (Status::Ok, _, ReplyData::Files(files)) => {
            let names: Vec<&str> = files.iter().map(|f| f.name.as_str()).collect();
            assert_eq!(names, vec!["match-1.athrp", "tuning.json"]);
            assert_eq!(files[0].size, 10);
        }
        other => panic!("{other:?}"),
    }

    match client.reply(ClientMessage::FileDownload { name: "match-1.athrp".into(), offset: 4, max_len: 3 }) {
        (Status::Ok, _, ReplyData::Chunk { eof, data }) => {
            assert!(!eof);
            assert_eq!(data, b"456");
        }
        other => panic!("{other:?}"),
    }
    match client.reply(ClientMessage::FileDownload { name: "match-1.athrp".into(), offset: 8, max_len: 100 }) {
        (Status::Ok, _, ReplyData::Chunk { eof, data }) => {
            assert!(eof);
            assert_eq!(data, b"89");
        }
        other => panic!("{other:?}"),
    }

    for bad in ["../secret", "a/b", "..", "", "C:evil"] {
        let (status, _, _) = client.reply(ClientMessage::FileDownload { name: bad.into(), offset: 0, max_len: 10 });
        assert_eq!(status, Status::Error, "{bad}");
        let (status, _, _) = client.reply(ClientMessage::FileDelete { name: bad.into() });
        assert_eq!(status, Status::Error, "{bad}");
    }

    let (status, _, _) = client.reply(ClientMessage::FileDelete { name: "tuning.json".into() });
    assert_eq!(status, Status::Ok);
    assert!(!root.join("tuning.json").exists());
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn bandwidth_budget_lowers_lowest_priority_rate_and_reports_it() {
    let (server, _) = server_with(ServerConfig { bandwidth_budget_bytes_per_sec: 200, ..ServerConfig::default() });
    let (udp, port) = udp_pair();
    let mut client = Client::connect(server.control_addr());
    client.hello(port);
    let first = client.subscribe("robot/turret/angle", Rate::Hz(500.0));
    let second = client.subscribe("robot/intake/**", Rate::Hz(500.0));
    drain_udp(&udp);

    // Six seconds of robot time with changing values, so both streams always have data to send.
    for i in 0..3000u64 {
        server.publish_cycle(i * 2000, &[angle(i as f64), Entry { signal_id: OTHER, value: Value::F64(i as f64) }]);
    }
    let status = server.status();
    let divisor = |id: u32| status.throttled.iter().find(|t| t.subscription_id == id).map_or(1, |t| t.divisor);
    assert!(divisor(second) > 1, "the newest subscription must be throttled");
    assert!(divisor(second) >= divisor(first), "lowest priority is throttled first");
    assert!(status.throttled.iter().all(|t| t.divisor > 1));
    assert_eq!(status.subscriptions, 2);
}

#[test]
fn server_rejects_hello_with_wrong_protocol_version() {
    let (server, _) = server();
    let mut client = Client::connect(server.control_addr());
    let (status, reason, _) = client.reply(ClientMessage::Hello { client_name: "old".into(), protocol_version: 1, udp_port: 1 });
    assert_eq!(status, Status::Error);
    assert_eq!(reason, "unsupported protocol version");
}
