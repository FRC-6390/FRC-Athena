use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use arcp_core::{
    SignalAccess, SignalDescriptor, SignalDurability, SignalPolicy, SignalType, SignalValue,
};
use arcp_dashboard::{ControlClient, DashboardState, TelemetrySubscription};
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
fn dashboard_receives_realtime_updates() {
    let server = ArcpServer::new(ArcpServerConfig {
        control_port: 0,
        realtime_port: 0,
        max_signals: 32,
    });
    server
        .register_signal(SignalDescriptor::telemetry(
            7,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Test/velocity",
        ))
        .expect("register signal");
    server.start().expect("start server");
    wait_for_ports(&server);

    let mut control =
        ControlClient::connect("127.0.0.1", server.control_port()).expect("connect control");
    let stats = control.server_stats().expect("server stats");
    assert!(
        stats.1.is_some() || cfg!(not(target_os = "linux")),
        "rss is expected on linux servers"
    );
    let manifest = control.manifest().expect("manifest");
    assert_eq!(manifest.len(), 1);

    let telemetry = TelemetrySubscription::bind_any().expect("bind telemetry");
    let local_port = telemetry.local_port().expect("telemetry port");
    control.subscribe(local_port).expect("subscribe");

    let state = Arc::new(Mutex::new(DashboardState::new(manifest)));
    let running = Arc::new(AtomicBool::new(true));
    let handle = telemetry
        .spawn(Arc::clone(&state), Arc::clone(&running))
        .expect("spawn telemetry thread");

    server.publish_f64(7, 4.2).expect("publish update");

    let deadline = Instant::now() + Duration::from_secs(2);
    let mut seen = false;
    while Instant::now() < deadline {
        if let Ok(guard) = state.lock() {
            if guard.value_for(7) == Some(&SignalValue::F64(4.2)) {
                seen = true;
                break;
            }
        }
        thread::sleep(Duration::from_millis(10));
    }

    running.store(false, Ordering::SeqCst);
    let _ = handle.join();
    let _ = control.unsubscribe(local_port);
    server.stop();

    assert!(seen, "dashboard did not observe published value");
}

#[test]
fn control_client_roundtrips_server_layout_profiles() {
    let server = ArcpServer::new(ArcpServerConfig {
        control_port: 0,
        realtime_port: 0,
        max_signals: 8,
    });
    server.start().expect("start server");
    wait_for_ports(&server);

    let mut control =
        ControlClient::connect("127.0.0.1", server.control_port()).expect("connect control");
    let payload = r#"{"format":"arcp.host.layout.export","version":1,"layout":{"activeTabId":"tab-1","tabs":[]}}"#;
    control
        .save_layout("demo", payload)
        .expect("save layout profile");
    let loaded = control.load_layout("demo").expect("load layout profile");
    assert_eq!(loaded, payload);
    let list = control.list_layouts().expect("list layout profiles");
    assert!(list.iter().any(|entry| entry == "demo"));

    server.stop();
}
