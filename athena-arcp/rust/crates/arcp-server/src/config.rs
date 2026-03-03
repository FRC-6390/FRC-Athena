#[derive(Debug, Clone)]
pub struct ArcpServerConfig {
    pub control_port: u16,
    pub realtime_port: u16,
    pub max_signals: u16,
}

impl Default for ArcpServerConfig {
    fn default() -> Self {
        Self {
            control_port: 5805,
            realtime_port: 5806,
            max_signals: 8192,
        }
    }
}
