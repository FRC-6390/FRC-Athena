use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, UdpSocket};
use std::thread;
use std::time::{Duration, Instant};

use arcp_core::{
    decode_event, decode_update, RuntimeEvent, SignalAccess, SignalDescriptor, SignalDurability,
    SignalPolicy, SignalType, SignalValue,
};
use arcp_server::{ArcpServer, ArcpServerConfig};

fn wait_for_ports(server: &ArcpServer) {
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        if server.control_port() != 0 && server.realtime_port() != 0 {
            return;
        }
        thread::sleep(Duration::from_millis(10));
    }
    panic!("server ports did not become available in time");
}

fn decode_event_batch(data: &[u8]) -> Vec<RuntimeEvent> {
    let mut events = Vec::new();
    let mut index = 0_usize;
    while index + 2 <= data.len() {
        let len = u16::from_le_bytes([data[index], data[index + 1]]) as usize;
        index += 2;
        if index + len > data.len() {
            break;
        }
        let event = decode_event(&data[index..index + len]).expect("decode event");
        events.push(event);
        index += len;
    }
    events
}

fn hex_encode(raw: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(raw.len().saturating_mul(2));
    for byte in raw {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn decode_hex_nibble(byte: u8) -> Result<u8, &'static str> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        b'A'..=b'F' => Ok(byte - b'A' + 10),
        _ => Err("invalid nibble"),
    }
}

fn hex_decode(raw: &str) -> Result<Vec<u8>, &'static str> {
    if raw.len() % 2 != 0 {
        return Err("hex payload must have even length");
    }
    let mut out = Vec::with_capacity(raw.len() / 2);
    let bytes = raw.as_bytes();
    let mut index = 0_usize;
    while index < bytes.len() {
        let hi = decode_hex_nibble(bytes[index])?;
        let lo = decode_hex_nibble(bytes[index + 1])?;
        out.push((hi << 4) | lo);
        index += 2;
    }
    Ok(out)
}

#[test]
fn control_set_and_action_emit_events() {
    let server = ArcpServer::new(ArcpServerConfig {
        control_port: 0,
        realtime_port: 0,
        max_signals: 64,
        nt4_bridge_enabled: false,
        nt4_unsecure_port: 5810,
    });
    server
        .register_signal(SignalDescriptor::telemetry(
            5,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Tuning/kP",
        ))
        .expect("register tunable");
    server
        .register_signal(SignalDescriptor::command(
            6,
            SignalType::Bool,
            "/Athena/Actions/reset",
        ))
        .expect("register action");
    server.start().expect("server start");
    wait_for_ports(&server);

    let mut control =
        TcpStream::connect(("127.0.0.1", server.control_port())).expect("control connect");
    let mut reader = BufReader::new(control.try_clone().expect("clone control"));
    let mut welcome = String::new();
    reader.read_line(&mut welcome).expect("read welcome");
    assert!(welcome.starts_with("WELCOME "));

    control.write_all(b"SET 5 12.75\n").expect("write set");
    control.flush().expect("flush set");
    let mut set_response = String::new();
    reader
        .read_line(&mut set_response)
        .expect("read set response");
    assert_eq!(set_response.trim(), "OK SET");

    control.write_all(b"ACTION 6\n").expect("write action");
    control.flush().expect("flush action");
    let mut action_response = String::new();
    reader
        .read_line(&mut action_response)
        .expect("read action response");
    assert_eq!(action_response.trim(), "OK ACTION");

    let deadline = Instant::now() + Duration::from_secs(2);
    let mut events = Vec::new();
    while Instant::now() < deadline && events.len() < 2 {
        let mut buf = [0_u8; 512];
        let written = server.poll_events_into(&mut buf);
        if written > 0 {
            events.extend(decode_event_batch(&buf[..written]));
        } else {
            thread::sleep(Duration::from_millis(10));
        }
    }
    assert!(events.contains(&RuntimeEvent::TunableSet {
        signal_id: 5,
        value: SignalValue::F64(12.75)
    }));
    assert!(events.contains(&RuntimeEvent::Action { signal_id: 6 }));

    server.stop();
}

#[test]
fn publishes_all_supported_types_over_udp() {
    let server = ArcpServer::new(ArcpServerConfig {
        control_port: 0,
        realtime_port: 0,
        max_signals: 128,
        nt4_bridge_enabled: false,
        nt4_unsecure_port: 5810,
    });
    let descriptors = [
        SignalDescriptor::telemetry(
            11,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/bool",
        ),
        SignalDescriptor::telemetry(
            12,
            SignalType::I64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/i64",
        ),
        SignalDescriptor::telemetry(
            13,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/f64",
        ),
        SignalDescriptor::telemetry(
            14,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/string",
        ),
        SignalDescriptor::telemetry(
            15,
            SignalType::BoolArray,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/bool_array",
        ),
        SignalDescriptor::telemetry(
            16,
            SignalType::I64Array,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/i64_array",
        ),
        SignalDescriptor::telemetry(
            17,
            SignalType::F64Array,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/f64_array",
        ),
        SignalDescriptor::telemetry(
            18,
            SignalType::StrArray,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/string_array",
        ),
    ];
    for descriptor in descriptors {
        server.register_signal(descriptor).expect("register signal");
    }
    server.start().expect("server start");
    wait_for_ports(&server);

    let udp_client = UdpSocket::bind("127.0.0.1:0").expect("udp bind");
    udp_client
        .set_read_timeout(Some(Duration::from_secs(2)))
        .expect("set timeout");
    let udp_port = udp_client.local_addr().expect("local addr").port();
    let mut control =
        TcpStream::connect(("127.0.0.1", server.control_port())).expect("control connect");
    let mut reader = BufReader::new(control.try_clone().expect("clone control"));
    let mut welcome = String::new();
    reader.read_line(&mut welcome).expect("read welcome");
    control
        .write_all(format!("SUB {}\n", udp_port).as_bytes())
        .expect("write sub");
    control.flush().expect("flush sub");
    let mut ack = String::new();
    reader.read_line(&mut ack).expect("read sub ack");
    assert_eq!(ack.trim(), "OK SUB");

    server.publish_bool(11, true).expect("publish bool");
    server.publish_i64(12, -7).expect("publish i64");
    server.publish_f64(13, 3.5).expect("publish f64");
    server
        .publish_string(14, "alpha".to_string())
        .expect("publish string");
    server
        .publish_bool_array(15, vec![true, false, true])
        .expect("publish bool[]");
    server
        .publish_i64_array(16, vec![1, 2, 3])
        .expect("publish i64[]");
    server
        .publish_f64_array(17, vec![1.0, 2.0, 3.5])
        .expect("publish f64[]");
    server
        .publish_string_array(18, vec!["a".to_string(), "b".to_string()])
        .expect("publish string[]");

    let mut received = Vec::new();
    while received.len() < 8 {
        let mut packet = [0_u8; 256];
        let (size, _) = udp_client.recv_from(&mut packet).expect("recv update");
        received.push(decode_update(&packet[..size]).expect("decode update"));
    }
    assert!(received.contains(&(11, SignalValue::Bool(true))));
    assert!(received.contains(&(12, SignalValue::I64(-7))));
    assert!(received.contains(&(13, SignalValue::F64(3.5))));
    assert!(received.contains(&(14, SignalValue::Str("alpha".to_string()))));
    assert!(received.contains(&(15, SignalValue::BoolArray(vec![true, false, true]))));
    assert!(received.contains(&(16, SignalValue::I64Array(vec![1, 2, 3]))));
    assert!(received.contains(&(17, SignalValue::F64Array(vec![1.0, 2.0, 3.5]))));
    assert!(received.contains(&(
        18,
        SignalValue::StrArray(vec!["a".to_string(), "b".to_string(),])
    )));

    server.stop();
}

#[test]
fn control_stats_reports_server_metrics() {
    let server = ArcpServer::new(ArcpServerConfig {
        control_port: 0,
        realtime_port: 0,
        max_signals: 8,
        nt4_bridge_enabled: false,
        nt4_unsecure_port: 5810,
    });
    server.start().expect("server start");
    wait_for_ports(&server);

    let mut control =
        TcpStream::connect(("127.0.0.1", server.control_port())).expect("control connect");
    let mut reader = BufReader::new(control.try_clone().expect("clone control"));
    let mut welcome = String::new();
    reader.read_line(&mut welcome).expect("read welcome");
    assert!(welcome.starts_with("WELCOME "));

    control.write_all(b"STATS\n").expect("write stats");
    control.flush().expect("flush stats");
    let mut stats = String::new();
    reader.read_line(&mut stats).expect("read stats");
    let stats = stats.trim();
    assert!(
        stats.starts_with("STATS "),
        "unexpected stats line: {stats}"
    );
    assert!(stats.contains("cpu="), "missing cpu token: {stats}");
    assert!(stats.contains("rss="), "missing rss token: {stats}");

    server.stop();
}

#[test]
fn control_layout_save_and_load_roundtrip() {
    let server = ArcpServer::new(ArcpServerConfig {
        control_port: 0,
        realtime_port: 0,
        max_signals: 8,
        nt4_bridge_enabled: false,
        nt4_unsecure_port: 5810,
    });
    server.start().expect("server start");
    wait_for_ports(&server);

    let mut control =
        TcpStream::connect(("127.0.0.1", server.control_port())).expect("control connect");
    let mut reader = BufReader::new(control.try_clone().expect("clone control"));
    let mut welcome = String::new();
    reader.read_line(&mut welcome).expect("read welcome");
    assert!(welcome.starts_with("WELCOME "));

    let payload = r#"{"format":"arcp.host.layout.export","version":1,"layout":{"activeTabId":"tab-1","tabs":[]}}"#;
    let payload_hex = hex_encode(payload.as_bytes());
    control
        .write_all(format!("LAYOUT_SAVE demo {payload_hex}\n").as_bytes())
        .expect("write layout save");
    control.flush().expect("flush layout save");
    let mut save_ack = String::new();
    reader.read_line(&mut save_ack).expect("read layout save");
    assert_eq!(save_ack.trim(), "OK LAYOUT_SAVE");

    control
        .write_all(b"LAYOUT_LOAD demo\n")
        .expect("write layout load");
    control.flush().expect("flush layout load");
    let mut load_line = String::new();
    reader.read_line(&mut load_line).expect("read layout load");
    let response = load_line.trim();
    assert!(
        response.starts_with("LAYOUT_DATA demo "),
        "unexpected layout load response: {response}"
    );
    let encoded = response
        .strip_prefix("LAYOUT_DATA demo ")
        .unwrap_or_default();
    let decoded = String::from_utf8(hex_decode(encoded).expect("decode hex")).expect("decode utf8");
    assert_eq!(decoded, payload);

    server.stop();
}
