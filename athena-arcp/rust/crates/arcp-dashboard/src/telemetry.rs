use std::io;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use arcp_core::decode_update;

use crate::state::DashboardState;

pub struct TelemetrySubscription {
    socket: UdpSocket,
}

impl TelemetrySubscription {
    pub fn bind_any() -> io::Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.set_read_timeout(Some(Duration::from_millis(100)))?;
        Ok(Self { socket })
    }

    pub fn local_port(&self) -> io::Result<u16> {
        Ok(self.socket.local_addr()?.port())
    }

    pub fn spawn(
        self,
        state: Arc<Mutex<DashboardState>>,
        running: Arc<AtomicBool>,
    ) -> io::Result<JoinHandle<()>> {
        let socket = self.socket;
        thread::Builder::new()
            .name("arcp-dashboard-telemetry".to_string())
            .spawn(move || {
                let mut packet = [0_u8; 1500];
                while running.load(Ordering::SeqCst) {
                    match socket.recv_from(&mut packet) {
                        Ok((size, _)) => {
                            if let Ok((signal_id, value)) = decode_update(&packet[..size]) {
                                if let Ok(mut guard) = state.lock() {
                                    guard.apply_update(signal_id, value);
                                }
                            }
                        }
                        Err(err)
                            if err.kind() == io::ErrorKind::WouldBlock
                                || err.kind() == io::ErrorKind::TimedOut =>
                        {
                            continue;
                        }
                        Err(_) => {
                            thread::sleep(Duration::from_millis(10));
                        }
                    }
                }
            })
    }
}
