//! End-to-end: a WebSocket client drives the browser bridge, which drives an in-process arcp-server v2.

use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use arcp_client::ws_bridge::serve;
use arcp_client::WriteReply;
use arcp_core::v2::record::{decode_stream, DictEntry, Dictionary, Record, StructSchema, DICT_FLAG_WRITABLE};
use arcp_core::v2::{Entry, Value};
use arcp_server::v2::{ArcpServerV2, RobotHandler, ServerConfig};
use serde_json::{json, Value as Json};
use tungstenite::Message;

const ANGLE: u64 = 1;
const SPEED: u64 = 4;
const LOCK: u64 = 5;
const WAIT: Duration = Duration::from_secs(5);

struct Robot {
    locked: AtomicBool,
    actions: Mutex<Vec<(u64, bool)>>,
}

impl RobotHandler for Robot {
    fn write(&self, signal_id: u64, value: Value) -> WriteReply {
        if signal_id == ANGLE && self.locked.load(Ordering::SeqCst) {
            WriteReply::rejected("rejected: locked (InMatch)", Value::F64(0.0))
        } else {
            WriteReply::accepted(value)
        }
    }

    fn action(&self, signal_id: u64, run: bool) -> Result<(), String> {
        self.actions.lock().unwrap().push((signal_id, run));
        if signal_id == SPEED {
            Ok(())
        } else {
            Err("no such action".into())
        }
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
            entry(SPEED, "robot/intake/speed", 3, DICT_FLAG_WRITABLE, 1, ""),
            entry(LOCK, "athena/locks/InMatch", 1, 0, 7, ""),
        ],
    }
}

/// A WebSocket client with helpers: JSON commands expect the reply with the same id; binary
/// messages are queued so record batches and events can be read in any order.
type Socket = tungstenite::WebSocket<tungstenite::stream::MaybeTlsStream<TcpStream>>;

struct Ws {
    socket: Socket,
    next_id: u64,
    records: Vec<Record>,
}

impl Ws {
    fn open(port: u16) -> Ws {
        let (socket, _) = tungstenite::connect(format!("ws://127.0.0.1:{port}/")).expect("bridge accepts client");
        if let tungstenite::stream::MaybeTlsStream::Plain(tcp) = socket.get_ref() {
            tcp.set_read_timeout(Some(Duration::from_millis(200))).unwrap();
        }
        Ws {
            socket,
            next_id: 1,
            records: Vec::new(),
        }
    }

    fn command(&mut self, mut body: Json) -> Json {
        let id = self.next_id;
        self.next_id += 1;
        body["id"] = json!(id);
        self.socket.send(Message::Text(body.to_string().into())).unwrap();
        let deadline = Instant::now() + WAIT;
        loop {
            assert!(Instant::now() < deadline, "no reply to {body}");
            match self.read() {
                Some(Message::Text(text)) => {
                    let value: Json = serde_json::from_str(text.as_str()).unwrap();
                    if value.get("id") == Some(&json!(id)) {
                        return value;
                    }
                }
                _ => {}
            }
        }
    }

    /// Reads one message; binary records are decoded into `self.records`.
    fn read(&mut self) -> Option<Message> {
        match self.socket.read() {
            Ok(Message::Binary(bytes)) => {
                self.records.extend(decode_stream(&bytes).expect("E1-W records decode"));
                Some(Message::Binary(bytes))
            }
            Ok(message) => Some(message),
            Err(tungstenite::Error::Io(e))
                if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) =>
            {
                None
            }
            Err(e) => panic!("socket error: {e}"),
        }
    }

    /// Reads until a record matches `pred`, or panics after `WAIT`.
    fn until(&mut self, what: &str, pred: impl Fn(&Record) -> bool) -> Record {
        let deadline = Instant::now() + WAIT;
        loop {
            if let Some(found) = self.records.iter().find(|r| pred(r)).cloned() {
                return found;
            }
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            self.read();
        }
    }
}

fn start_robot() -> (ArcpServerV2, Arc<Robot>) {
    let robot = Arc::new(Robot {
        locked: AtomicBool::new(true),
        actions: Mutex::new(Vec::new()),
    });
    let server = ArcpServerV2::start(ServerConfig::ephemeral(), robot.clone()).expect("robot server starts");
    server.publish_dictionary(
        dictionary(),
        vec![StructSchema {
            id: 1,
            type_name: "Pose2d".into(),
            schema: "x".into(),
        }],
    );
    (server, robot)
}

fn start_bridge(allow: Vec<String>) -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    thread::spawn(move || {
        let _ = serve(listener, allow);
    });
    port
}

#[test]
fn bridge_drives_robot_end_to_end() {
    let (robot_server, _robot) = start_robot();
    let bridge_port = start_bridge(Vec::new());
    let mut ws = Ws::open(bridge_port);

    // Status before connecting is a disconnected link, not an error.
    let status = ws.command(json!({ "cmd": "status" }));
    assert_eq!(status["ok"], json!(true));
    assert_eq!(status["result"]["connected"], json!(false));

    // Commands other than connect fail until a robot is connected.
    let early = ws.command(json!({ "cmd": "write", "path": "robot/intake/speed", "value": 1.0 }));
    assert_eq!(early["ok"], json!(false));

    let connect =
        ws.command(json!({ "cmd": "connect", "host": "127.0.0.1", "port": robot_server.control_addr().port() }));
    assert_eq!(connect["ok"], json!(true), "{connect}");
    assert!(connect["result"]["sessionId"].as_u64().unwrap() > 0);

    // The dictionary is pushed as records; a late subscriber gets it again as a snapshot.
    let dict = ws.until("dictionary record", |r| matches!(r, Record::Dictionary(_)));
    let Record::Dictionary(d) = dict else { unreachable!() };
    // The server adds its own athena/arcp/status signals to the three robot signals.
    assert!(d.entries.iter().any(|e| e.path == "robot/intake/speed"));
    assert!(d.entries.iter().any(|e| e.path.starts_with("athena/arcp/status/")));

    let sub = ws.command(json!({ "cmd": "subscribe", "pattern": "robot/**", "hz": 50.0 }));
    assert_eq!(sub["ok"], json!(true), "{sub}");

    robot_server.publish_cycle(
        1_000,
        &[Entry {
            signal_id: ANGLE,
            value: Value::F64(1.5),
        }],
    );
    let frame = ws.until("frame with angle 1.5", |r| match r {
        Record::Frame(f) => f
            .entries
            .iter()
            .any(|e| e.signal_id == ANGLE && e.value == Value::F64(1.5)),
        _ => false,
    });
    assert!(matches!(frame, Record::Frame(_)));

    // A rejected write is data: ok with accepted false and the robot's reason.
    let rejected = ws.command(json!({ "cmd": "write", "path": "robot/turret/angle", "value": 9.0 }));
    assert_eq!(rejected["ok"], json!(true), "{rejected}");
    assert_eq!(rejected["result"]["accepted"], json!(false));
    assert_eq!(rejected["result"]["reason"], json!("rejected: locked (InMatch)"));
    assert_eq!(rejected["result"]["actualValue"], json!(0.0));

    // An accepted write by signal id; the integer 3 becomes a float because the signal is F64.
    let accepted = ws.command(json!({ "cmd": "write", "signalId": SPEED, "value": 3 }));
    assert_eq!(accepted["result"]["accepted"], json!(true), "{accepted}");
    assert_eq!(accepted["result"]["actualValue"], json!(3.0));

    let run = ws.command(json!({ "cmd": "action", "signalId": SPEED, "run": true }));
    assert_eq!(run["result"]["accepted"], json!(true), "{run}");
    let cancel = ws.command(json!({ "cmd": "action", "path": "robot/turret/angle", "run": false }));
    assert_eq!(cancel["result"]["accepted"], json!(false), "{cancel}");
    assert!(cancel["result"]["reason"].as_str().unwrap().contains("no such action"));

    let unknown = ws.command(json!({ "cmd": "write", "path": "robot/nope", "value": 1 }));
    assert_eq!(unknown["ok"], json!(false));
    assert!(unknown["error"].as_str().unwrap().contains("unknown signal"));

    let garbage = ws.command(json!({ "cmd": "fly" }));
    assert_eq!(garbage["ok"], json!(false));

    let status = ws.command(json!({ "cmd": "status" }));
    assert_eq!(status["result"]["connected"], json!(true));
    assert!(status["result"]["frames"].as_u64().unwrap() >= 1);

    let disconnect = ws.command(json!({ "cmd": "disconnect" }));
    assert_eq!(disconnect["ok"], json!(true));
    let status = ws.command(json!({ "cmd": "status" }));
    assert_eq!(status["result"]["connected"], json!(false));
}

#[test]
fn raw_text_that_is_not_json_gets_an_error_reply() {
    let bridge_port = start_bridge(Vec::new());
    let mut ws = Ws::open(bridge_port);
    ws.socket.send(Message::Text("not json".into())).unwrap();
    let deadline = Instant::now() + WAIT;
    loop {
        assert!(Instant::now() < deadline);
        if let Some(Message::Text(text)) = ws.read() {
            let value: Json = serde_json::from_str(text.as_str()).unwrap();
            assert_eq!(value["ok"], json!(false));
            assert!(value["error"].as_str().unwrap().starts_with("bad JSON"));
            break;
        }
    }
}

#[test]
fn foreign_origins_are_refused_and_allow_list_is_honoured() {
    use tungstenite::client::IntoClientRequest;
    let port = start_bridge(vec!["https://dash.example".into()]);
    let connect_with_origin = |origin: &str| {
        let mut request = format!("ws://127.0.0.1:{port}/")
            .into_client_request()
            .expect("valid url");
        request.headers_mut().insert("origin", origin.parse().unwrap());
        tungstenite::connect(request).map(|_| ())
    };
    assert!(connect_with_origin("https://evil.example").is_err());
    assert!(connect_with_origin("http://localhost:1420").is_ok());
    assert!(connect_with_origin("https://dash.example").is_ok());
}
