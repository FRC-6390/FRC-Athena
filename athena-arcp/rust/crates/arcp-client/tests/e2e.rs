//! End-to-end: the client against an in-process arcp-server v2 over real TCP and UDP on localhost.

use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use arcp_client::{Client, ClientConfig, ClientError, Event, Rate, Record, Status, Value, WriteReply};
use arcp_core::v2::record::{decode_stream, encode_stream, DictEntry, Dictionary, Frame, Marker, Metadata, StructSchema, DICT_FLAG_WRITABLE};
use arcp_core::v2::wire::Writer;
use arcp_core::v2::{Entry, Record as WireRecord};
use arcp_server::v2::{ArcpServerV2, RobotHandler, ServerConfig};

const ANGLE: u64 = 1;
const OTHER: u64 = 4;
const LOCK: u64 = 5;
const WAIT: Duration = Duration::from_secs(5);

/// Robot stand-in: rejects writes to ANGLE while the InMatch lock is held, and answers backfill
/// with one record group per requested cycle.
struct TestRobot {
    locked: AtomicBool,
    actions: Mutex<Vec<(u64, bool)>>,
}

impl RobotHandler for TestRobot {
    fn write(&self, signal_id: u64, value: Value) -> WriteReply {
        if signal_id == ANGLE && self.locked.load(Ordering::SeqCst) {
            WriteReply::rejected("rejected: locked (InMatch)", Value::F64(0.0))
        } else {
            WriteReply::accepted(value)
        }
    }

    fn action(&self, signal_id: u64, run: bool) -> Result<(), String> {
        self.actions.lock().unwrap().push((signal_id, run));
        if signal_id == OTHER { Ok(()) } else { Err("no such action".into()) }
    }

    fn backfill(&self, from: u64, to: u64) -> Result<Vec<u8>, String> {
        let mut out = Writer::new();
        for seq in from..=to {
            let group = encode_stream(&[WireRecord::Frame(Frame::new(
                false,
                seq * 1_000,
                seq as u32,
                vec![Entry { signal_id: ANGLE, value: Value::F64(seq as f64) }],
            ))]);
            out.varint(group.len() as u64).bytes(&group);
        }
        Ok(out.into_vec())
    }
}

fn dictionary() -> Dictionary {
    let entry = |id: u64, path: &str, tag: u8, flags: u8, role: u8, lock: &str| DictEntry {
        signal_id: id,
        path: path.into(),
        type_tag: tag,
        unit: String::new(),
        role,
        flags,
        lock_group: lock.into(),
        struct_schema_id: 0,
    };
    Dictionary {
        entries: vec![
            entry(ANGLE, "robot/turret/angle", 3, DICT_FLAG_WRITABLE, 1, "InMatch"),
            entry(OTHER, "robot/intake/speed", 3, DICT_FLAG_WRITABLE, 1, ""),
            entry(LOCK, "athena/locks/InMatch", 1, 0, 7, ""),
        ],
    }
}

fn start_server(file_root: Option<PathBuf>) -> (ArcpServerV2, Arc<TestRobot>) {
    let robot = Arc::new(TestRobot { locked: AtomicBool::new(true), actions: Mutex::new(Vec::new()) });
    let config = ServerConfig { file_root, ..ServerConfig::ephemeral() };
    let server = ArcpServerV2::start(config, robot.clone()).expect("server starts");
    server.publish_dictionary(dictionary(), vec![StructSchema { id: 1, type_name: "Pose2d".into(), schema: "x".into() }]);
    (server, robot)
}

/// Waits for the first event matching `pred`, discarding others, or panics after `WAIT`.
fn next_matching(rx: &Receiver<Event>, what: &str, mut pred: impl FnMut(&Event) -> bool) -> Event {
    let deadline = Instant::now() + WAIT;
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok(ev) if pred(&ev) => return ev,
            Ok(_) => {}
            Err(e) => panic!("timed out waiting for {what}: {e}"),
        }
    }
}

fn has_angle(value: f64) -> impl Fn(&Event) -> bool {
    move |ev| match ev {
        Event::Frame(f) => f.entries.iter().any(|e| e.signal_id == ANGLE && e.value == Value::F64(value)),
        _ => false,
    }
}

fn publish_angle(server: &ArcpServerV2, cycle: u64, value: f64) {
    server.publish_cycle(cycle * 1_000, &[Entry { signal_id: ANGLE, value: Value::F64(value) }]);
}

fn connect(server: &ArcpServerV2, replay: bool) -> (Client, Receiver<Event>) {
    let mut config = ClientConfig::new("127.0.0.1", server.control_addr().port());
    config.replay = replay;
    config.request_timeout = Duration::from_secs(3);
    Client::connect(config).expect("client connects")
}

#[test]
fn commands_status_files_and_records_round_trip() {
    let dir = std::env::temp_dir().join(format!("arcp-client-e2e-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("a.athrp"), b"hello replay").unwrap();
    let (server, robot) = start_server(Some(dir.clone()));
    let (client, rx) = connect(&server, true);

    // Dictionary and schema arrive on connect; the model answers lookups.
    next_matching(&rx, "dictionary", |e| matches!(e, Event::Dictionary { .. }));
    assert!(client.dictionary().entries.iter().any(|e| e.path == "robot/turret/angle" && e.writable()));
    assert_eq!(client.signal_id("robot/turret/angle"), Some(ANGLE));
    assert_eq!(client.schemas().len(), 1);
    assert!(client.status().session_id > 0);

    // Subscribe, then a published cycle reaches the client as a frame.
    client.subscribe("robot/turret/**", Rate::OnChange).expect("subscribe");
    publish_angle(&server, 1, 1.5);
    next_matching(&rx, "frame", has_angle(1.5));
    assert_eq!(client.value(ANGLE), Some(Value::F64(1.5)));

    // The server status arrives as system signals, once a second, with or without subscriptions.
    let deadline = Instant::now() + WAIT;
    while client.server_status().is_none() {
        assert!(Instant::now() < deadline, "server status never arrived");
        thread::sleep(Duration::from_millis(10));
    }
    let status = client.server_status().unwrap();
    assert_eq!(status.clients, 1);
    assert!(!status.throttled && status.throttled_patterns.is_empty());

    // Locked: the robot rejects and reports the value it still holds (E5).
    let rejected = client.write(ANGLE, Value::F64(9.0)).expect("write answered");
    assert!(!rejected.accepted);
    assert_eq!(rejected.reason, "rejected: locked (InMatch)");
    assert_eq!(rejected.actual_value, Value::F64(0.0));
    assert_eq!(client.lock_state("InMatch"), None, "lock signal not published yet");

    // Unlocked elsewhere: an accepted write reports the applied value.
    let accepted = client.write(OTHER, Value::F64(2.0)).expect("write answered");
    assert!(accepted.accepted);
    assert_eq!(accepted.actual_value, Value::F64(2.0));

    // Actions: accepted and rejected.
    client.action_run(OTHER).expect("action runs");
    client.action_cancel(OTHER).expect("cancel runs");
    match client.action_run(ANGLE) {
        Err(ClientError::Server { status: Status::Rejected, reason }) => assert_eq!(reason, "no such action"),
        other => panic!("expected rejection, got {other:?}"),
    }
    assert_eq!(robot.actions.lock().unwrap().len(), 3);

    // Time sync keeps multiple samples and reports a finite RTT.
    let est = client.time_sync(5).expect("time sync");
    assert_eq!(est.samples, 5);
    assert!(est.rtt_micros >= 0.0 && est.rtt_micros.is_finite());
    assert!(client.status().rtt_micros.is_some());

    // Files: list, download, delete.
    let files = client.file_list().expect("list");
    assert!(files.iter().any(|f| f.name == "a.athrp" && f.size == 12));
    assert_eq!(client.file_download("a.athrp", 1024).expect("download"), b"hello replay");
    assert!(client.file_download("a.athrp", 4).is_err(), "size cap enforced");
    client.file_delete("a.athrp").expect("delete");
    assert!(!client.file_list().unwrap().iter().any(|f| f.name == "a.athrp"));

    // Replay records (non-frame) arrive as events once subscribed.
    server.marker(Marker { timestamp_micros: 42, kind: 1, message: "boot".into(), signal_id: 0 });
    let marker = next_matching(&rx, "marker", |e| matches!(e, Event::Record(Record::Marker(_))));
    assert!(matches!(marker, Event::Record(Record::Marker(m)) if m.message == "boot"));
    server.publish_records(&encode_stream(&[WireRecord::Metadata(Metadata { pairs: vec![("event".into(), "practice".into())] })]));
    next_matching(&rx, "metadata", |e| matches!(e, Event::Record(Record::Metadata(_))));

    // Direct backfill returns the decoded records.
    let records = client.backfill(3, 4).expect("backfill");
    let seqs: Vec<u32> = records
        .iter()
        .filter_map(|r| match r {
            Record::Frame(f) => Some(f.sequence),
            _ => None,
        })
        .collect();
    assert_eq!(seqs, vec![3, 4]);

    assert!(client.status().frames >= 1);
    client.close();
    next_matching(&rx, "disconnect", |e| matches!(e, Event::Disconnected { .. }));
    assert!(!client.is_connected());
    let _ = std::fs::remove_dir_all(&dir);
}

/// UDP relay between the server and the client. It drops the third angle-bearing datagram and
/// forwards everything else to `target` (the client's bound port, set after connect).
struct LossyRelay {
    port: u16,
    target: Arc<AtomicU16>,
    dropped_seq: Arc<AtomicU32>,
    stop: Arc<AtomicBool>,
}

impl LossyRelay {
    fn spawn() -> LossyRelay {
        let relay = UdpSocket::bind("127.0.0.1:0").unwrap();
        relay.set_read_timeout(Some(Duration::from_millis(100))).unwrap();
        let port = relay.local_addr().unwrap().port();
        let target = Arc::new(AtomicU16::new(0));
        let dropped_seq = Arc::new(AtomicU32::new(u32::MAX));
        let stop = Arc::new(AtomicBool::new(false));
        let (t, d, s) = (target.clone(), dropped_seq.clone(), stop.clone());
        thread::spawn(move || {
            let mut buf = [0u8; 65_536];
            let mut frames_seen = 0u32;
            while !s.load(Ordering::SeqCst) {
                let Ok((n, _)) = relay.recv_from(&mut buf) else { continue };
                let target_port = t.load(Ordering::SeqCst);
                if target_port == 0 {
                    continue;
                }
                // Only datagrams carrying the angle signal count; status frames pass through.
                let seq = decode_stream(&buf[..n]).ok().and_then(|recs| {
                    recs.into_iter().find_map(|r| match r {
                        WireRecord::Frame(f) if f.entries.iter().any(|e| e.signal_id == ANGLE) => Some(f.sequence),
                        _ => None,
                    })
                });
                if let Some(seq) = seq {
                    frames_seen += 1;
                    if frames_seen == 3 {
                        d.store(seq, Ordering::SeqCst);
                        continue;
                    }
                }
                let _ = relay.send_to(&buf[..n], SocketAddr::from(([127, 0, 0, 1], target_port)));
            }
        });
        LossyRelay { port, target, dropped_seq, stop }
    }
}

impl Drop for LossyRelay {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
    }
}

#[test]
fn lost_frame_is_detected_and_backfilled() {
    let (server, _robot) = start_server(None);
    let relay = LossyRelay::spawn();
    let mut config = ClientConfig::new("127.0.0.1", server.control_addr().port());
    config.udp_advertise = Some(relay.port);
    config.request_timeout = Duration::from_secs(3);
    let (client, rx) = Client::connect(config).expect("client connects");
    relay.target.store(client.udp_port(), Ordering::SeqCst);
    next_matching(&rx, "dictionary", |e| matches!(e, Event::Dictionary { .. }));

    client.subscribe("robot/turret/**", Rate::OnChange).expect("subscribe");
    for cycle in 1..=6u64 {
        publish_angle(&server, cycle, cycle as f64 + 0.5);
        thread::sleep(Duration::from_millis(20));
    }

    // Collect what arrives until the backfill answer, so we can see which cycle went missing.
    let mut angles = Vec::new();
    let mut gap = None;
    let mut backfill = None;
    let deadline = Instant::now() + WAIT;
    // Keep reading until the backfill answer, then briefly longer for the cycles after it.
    while backfill.is_none() || Instant::now() < deadline.min(Instant::now() + Duration::from_millis(300)) {
        let left = deadline.saturating_duration_since(Instant::now());
        let Ok(ev) = rx.recv_timeout(left.min(Duration::from_millis(300))) else { break };
        match ev {
            Event::Frame(f) => {
                if let Some(e) = f.entries.iter().find(|e| e.signal_id == ANGLE) {
                    if let Value::F64(v) = e.value {
                        angles.push(v);
                    }
                }
            }
            Event::Gap { from, to, lost } => gap = Some((from, to, lost)),
            Event::Backfill { from, to, result } => backfill = Some((from, to, result)),
            _ => {}
        }
    }

    let dropped = relay.dropped_seq.load(Ordering::SeqCst);
    assert_ne!(dropped, u32::MAX, "relay dropped a frame");
    assert_eq!(gap, Some((dropped, dropped, 1)));
    let (from, to, result) = backfill.unwrap();
    assert_eq!((from, to), (u64::from(dropped), u64::from(dropped)));
    let records = result.expect("backfill succeeds");
    assert!(records.iter().any(|r| matches!(r, Record::Frame(f) if f.sequence == dropped)));

    // The cycle whose frame was dropped never arrived; the others did, in order.
    assert!(!angles.contains(&3.5), "dropped cycle must not arrive: {angles:?}");
    assert!(angles.contains(&4.5) && angles.contains(&6.5), "{angles:?}");

    let status = client.status();
    assert_eq!((status.gaps, status.lost_cycles, status.backfills), (1, 1, 1));
    client.close();
}
