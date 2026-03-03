#[derive(Debug, Clone)]
pub struct ArcpServerConfig {
    pub control_port: u16,
    pub realtime_port: u16,
    pub max_signals: u16,
    pub nt4_bridge_enabled: bool,
    pub nt4_unsecure_port: u16,
}

impl Default for ArcpServerConfig {
    fn default() -> Self {
        Self {
            control_port: 5805,
            realtime_port: 5806,
            max_signals: 8192,
            nt4_bridge_enabled: true,
            nt4_unsecure_port: 5810,
        }
    }
}
