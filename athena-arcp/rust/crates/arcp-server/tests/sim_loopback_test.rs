use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, UdpSocket};
use std::thread;
use std::time::{Duration, Instant};

use arcp_core::{
    decode_update, SignalAccess, SignalDescriptor, SignalDurability, SignalPolicy, SignalType,
    SignalValue,
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

#[test]
fn loopback_publish_reaches_udp_subscriber() {
    let server = ArcpServer::new(ArcpServerConfig {
        control_port: 0,
        realtime_port: 0,
        max_signals: 64,
    });
    server
        .register_signal(SignalDescriptor::telemetry(
            17,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/value",
        ))
        .expect("signal registration");

    server.start().expect("server should start");
    wait_for_ports(&server);

    let udp_client = UdpSocket::bind("127.0.0.1:0").expect("udp bind");
    udp_client
        .set_read_timeout(Some(Duration::from_secs(2)))
        .expect("set timeout");
    let udp_port = udp_client.local_addr().expect("local addr").port();

    let mut control =
        TcpStream::connect(("127.0.0.1", server.control_port())).expect("control connect");
    control
        .set_read_timeout(Some(Duration::from_secs(2)))
        .expect("control timeout");
    let mut control_reader = BufReader::new(control.try_clone().expect("clone control"));

    let mut welcome = String::new();
    control_reader
        .read_line(&mut welcome)
        .expect("read welcome");
    assert!(welcome.starts_with("WELCOME "));

    control
        .write_all(format!("SUB {}\n", udp_port).as_bytes())
        .expect("write sub");
    control.flush().expect("flush sub");

    let mut ack = String::new();
    control_reader.read_line(&mut ack).expect("read ack");
    assert_eq!(ack.trim(), "OK SUB");

    server
        .publish_f64(17, 123.456)
        .expect("publish should work");

    let mut packet = [0_u8; 128];
    let (size, _) = udp_client.recv_from(&mut packet).expect("recv packet");
    let (signal_id, value) = decode_update(&packet[..size]).expect("received packet should decode");
    assert_eq!(signal_id, 17);
    assert_eq!(value, SignalValue::F64(123.456));

    server.stop();
}
