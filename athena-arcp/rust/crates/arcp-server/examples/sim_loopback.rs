use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, UdpSocket};
use std::thread;
use std::time::{Duration, Instant};

use arcp_core::{
    decode_update, SignalAccess, SignalDescriptor, SignalDurability, SignalPolicy, SignalType,
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

fn main() {
    let server = ArcpServer::new(ArcpServerConfig {
        control_port: 0,
        realtime_port: 0,
        max_signals: 128,
    });
    server
        .register_signal(SignalDescriptor::telemetry(
            1,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Example/value",
        ))
        .expect("register signal");
    server.start().expect("server start");
    wait_for_ports(&server);

    println!(
        "ARCP server running on control={} realtime={}",
        server.control_port(),
        server.realtime_port()
    );

    let udp_client = UdpSocket::bind("127.0.0.1:0").expect("udp bind");
    udp_client
        .set_read_timeout(Some(Duration::from_secs(2)))
        .expect("set timeout");
    let udp_port = udp_client.local_addr().expect("udp addr").port();

    let mut control =
        TcpStream::connect(("127.0.0.1", server.control_port())).expect("control connect");
    let mut control_reader = BufReader::new(control.try_clone().expect("clone control"));
    let mut welcome = String::new();
    control_reader
        .read_line(&mut welcome)
        .expect("read welcome");
    println!("control -> {}", welcome.trim());

    control
        .write_all(format!("SUB {}\n", udp_port).as_bytes())
        .expect("write sub");
    control.flush().expect("flush sub");
    let mut ack = String::new();
    control_reader.read_line(&mut ack).expect("read ack");
    println!("control -> {}", ack.trim());

    for i in 0..5 {
        let value = (i as f64) * 10.0;
        server.publish_f64(1, value).expect("publish f64");
        let mut packet = [0_u8; 64];
        let (size, _) = udp_client.recv_from(&mut packet).expect("recv");
        let (signal_id, decoded) = decode_update(&packet[..size]).expect("decode");
        println!("udp <- signal={} value={:?}", signal_id, decoded);
    }

    server.stop();
    println!("ARCP simulation complete");
}
