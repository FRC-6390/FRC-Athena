use std::thread;
use std::time::{Duration, Instant};

use arcp_core::{
    decode_event, PersistenceScope, RuntimeEvent, SignalAccess, SignalDescriptor, SignalDurability,
    SignalPolicy, SignalType, SignalValue,
};
use arcp_server::{ArcpServer, ArcpServerConfig};

const LAYOUT_PROFILE_SHOWCASE: &str = "sim-all-layouts";

const SIG_HR_BOOL: u16 = 1;
const SIG_HR_I64: u16 = 2;
const SIG_HR_F64: u16 = 3;
const SIG_HR_STR: u16 = 4;
const SIG_HR_BOOL_ARRAY: u16 = 5;
const SIG_HR_I64_ARRAY: u16 = 6;
const SIG_HR_F64_ARRAY: u16 = 7;
const SIG_HR_STR_ARRAY: u16 = 8;

const SIG_OC_BOOL: u16 = 11;
const SIG_OC_I64: u16 = 12;
const SIG_OC_F64: u16 = 13;
const SIG_OC_STR: u16 = 14;
const SIG_OC_BOOL_ARRAY: u16 = 15;
const SIG_OC_I64_ARRAY: u16 = 16;
const SIG_OC_F64_ARRAY: u16 = 17;
const SIG_OC_STR_ARRAY: u16 = 18;

const SIG_TUNABLE_BOOL: u16 = 21;
const SIG_TUNABLE_I64: u16 = 22;
const SIG_TUNABLE_F64: u16 = 23;
const SIG_TUNABLE_STR: u16 = 24;
const SIG_TUNABLE_BOOL_ARRAY: u16 = 25;
const SIG_TUNABLE_I64_ARRAY: u16 = 26;
const SIG_TUNABLE_F64_ARRAY: u16 = 27;
const SIG_TUNABLE_STR_ARRAY: u16 = 28;

const SIG_CMD_RESET: u16 = 40;
const SIG_CMD_TOGGLE_ENABLED: u16 = 41;

const SIG_MATCH_TIME: u16 = 50;
const SIG_BATTERY_VOLTAGE: u16 = 51;
const SIG_ROBOT_POSE: u16 = 52;
const SIG_TRAJECTORY_POINTS: u16 = 53;
const SIG_SUBSYSTEM_HEALTH: u16 = 54;
const SIG_CAMERA_OVERLAY: u16 = 55;
const SIG_MODE_OPTIONS: u16 = 56;
const SIG_SELECTED_MODE: u16 = 57;
const SIG_CMD_CYCLE_MODE: u16 = 58;
const SIG_ROBOT_X: u16 = 59;
const SIG_ROBOT_Y: u16 = 60;
const SIG_ROBOT_HEADING_DEG: u16 = 61;
const SIG_ROBOT_STATE: u16 = 62;
const SIG_GAME_STATE: u16 = 63;
const SIG_CAMERA_STREAM_URL: u16 = 64;
const SIG_CAMERA_TARGETS: u16 = 65;
const SIG_CAMERA_DETECTIONS: u16 = 66;
const SIG_GRAPH_CHANNEL_A: u16 = 67;
const SIG_GRAPH_CHANNEL_B: u16 = 68;
const SIG_GRAPH_CHANNEL_C: u16 = 69;
const SIG_PID_DRIVE_KP: u16 = 70;
const SIG_PID_DRIVE_KI: u16 = 71;
const SIG_PID_DRIVE_KD: u16 = 72;
const SIG_PID_DRIVE_KS: u16 = 73;
const SIG_PID_DRIVE_KV: u16 = 74;
const SIG_PID_DRIVE_KA: u16 = 75;
const SIG_PID_DRIVE_SETPOINT: u16 = 76;
const SIG_PID_ARM_KP: u16 = 77;
const SIG_PID_ARM_KI: u16 = 78;
const SIG_PID_ARM_KD: u16 = 79;
const SIG_PID_ARM_KG: u16 = 80;
const SIG_PID_ARM_KS: u16 = 81;
const SIG_PID_ARM_KV: u16 = 82;
const SIG_PID_ARM_KA: u16 = 83;
const SIG_PID_ARM_SETPOINT: u16 = 84;
const SIG_FIELD_IMAGE_URL: u16 = 85;
const SIG_MECH2D_JSON: u16 = 86;
const SIG_SWERVE_MODULES: u16 = 87;
const SIG_SWERVE_FL_ANGLE_DEG: u16 = 88;
const SIG_SWERVE_FL_SPEED_MPS: u16 = 89;
const SIG_SWERVE_FR_ANGLE_DEG: u16 = 90;
const SIG_SWERVE_FR_SPEED_MPS: u16 = 91;
const SIG_SWERVE_BL_ANGLE_DEG: u16 = 92;
const SIG_SWERVE_BL_SPEED_MPS: u16 = 93;
const SIG_SWERVE_BR_ANGLE_DEG: u16 = 94;
const SIG_SWERVE_BR_SPEED_MPS: u16 = 95;
const SIG_DIFF_LEFT_SPEED_MPS: u16 = 96;
const SIG_DIFF_RIGHT_SPEED_MPS: u16 = 97;
const SIG_DIFF_HEADING_DEG: u16 = 98;

const SIG_NT4_PERF_STATUS_IS_SIMULATION: u16 = 120;
const SIG_NT4_PERF_STATUS_IS_DS_ATTACHED: u16 = 121;
const SIG_NT4_PERF_STATUS_IS_FMS_ATTACHED: u16 = 122;
const SIG_NT4_PERF_LOOP_PERIOD_SEC: u16 = 123;
const SIG_NT4_PERF_LOOP_TOTAL_MS: u16 = 124;
const SIG_NT4_PERF_LOOP_UTILIZATION_PERCENT: u16 = 125;
const SIG_NT4_PERF_POWER_BATTERY_VOLTAGE: u16 = 126;
const SIG_NT4_PERF_POWER_INPUT_CURRENT_AMPS: u16 = 127;
const SIG_NT4_PERF_POWER_CPU_TEMP_CELSIUS: u16 = 128;
const SIG_NT4_PERF_MEMORY_HEAP_USED_BYTES: u16 = 129;
const SIG_NT4_PERF_MEMORY_HEAP_COMMITTED_BYTES: u16 = 130;
const SIG_NT4_AUTO_ROUTINE_COUNT: u16 = 131;
const SIG_NT4_AUTO_FOLLOWER_ENABLED: u16 = 132;
const SIG_NT4_AUTO_SELECTED_ID: u16 = 133;
const SIG_NT4_AUTO_SELECTED_DISPLAY_NAME: u16 = 134;
const SIG_NT4_AUTO_HAS_TRAJECTORY: u16 = 135;
const SIG_NT4_LOCALIZATION_HEALTH_POSE_JUMP_METERS: u16 = 136;
const SIG_NT4_LOCALIZATION_HEALTH_DRIFT_RATE_MPS: u16 = 137;
const SIG_NT4_LOCALIZATION_HEALTH_VISION_ACCEPT_RATE: u16 = 138;
const SIG_NT4_LOCALIZATION_HEALTH_SLIP_ACTIVE: u16 = 139;
const SIG_NT4_LOCALIZATION_HEALTH_SLIP_MODE: u16 = 140;
const SIG_NT4_VISION_CAMERA_COUNT: u16 = 141;
const SIG_NT4_VISION_HAS_LOCALIZATION: u16 = 142;
const SIG_NT4_CONFIG_BASE_URL: u16 = 143;
const SIG_NT4_CONFIG_DIAGNOSTICS_URL: u16 = 144;
const SIG_NT4_DRIVETRAIN_ROBOTSPEEDS_DRIVER_VX_MPS: u16 = 145;
const SIG_NT4_DRIVETRAIN_ROBOTSPEEDS_DRIVER_VY_MPS: u16 = 146;
const SIG_NT4_DRIVETRAIN_ROBOTSPEEDS_DRIVER_OMEGA_RAD_PER_SEC: u16 = 147;
const SIG_NT4_DRIVETRAIN_DRIFT_CORRECTION_ENABLED: u16 = 148;
const SIG_NT4_DRIVETRAIN_DRIFT_CORRECTION_DESIRED_HEADING_DEG: u16 = 149;

const MODE_OPTIONS: [&str; 4] = ["disabled", "tele", "auto", "test"];
const ROBOT_STATE_SEQUENCE: [&str; 7] = [
    "disabled", "enabled", "test", "auto", "tele", "sim", "endgame",
];
const CAMERA_STREAM_DATA_URI: &str = "data:image/svg+xml,%3Csvg%20xmlns='http://www.w3.org/2000/svg'%20viewBox='0%200%20640%20360'%3E%3Cdefs%3E%3ClinearGradient%20id='g'%20x1='0'%20x2='1'%20y1='0'%20y2='1'%3E%3Cstop%20offset='0'%20stop-color='%230f172a'/%3E%3Cstop%20offset='1'%20stop-color='%231e293b'/%3E%3C/linearGradient%3E%3C/defs%3E%3Crect%20width='640'%20height='360'%20fill='url(%23g)'/%3E%3Cpath%20d='M-30%20290%20L180%20145%20L360%20220%20L670%20110'%20stroke='%23334155'%20stroke-width='38'%20fill='none'/%3E%3Ctext%20x='34'%20y='66'%20fill='%23fca5a5'%20font-size='42'%20font-family='monospace'%3EARCP%20Camera%3C/text%3E%3Ctext%20x='36'%20y='325'%20fill='%2394a3b8'%20font-size='20'%20font-family='monospace'%3ESimulated%20Stream%3C/text%3E%3C/svg%3E";
const FIELD_IMAGE_DATA_URI: &str = "data:image/svg+xml,%3Csvg%20xmlns='http://www.w3.org/2000/svg'%20viewBox='0%200%201654%20802'%3E%3Crect%20width='1654'%20height='802'%20fill='%230b1220'/%3E%3Crect%20x='12'%20y='12'%20width='1630'%20height='778'%20fill='%2312182a'%20stroke='%23334155'%20stroke-width='6'/%3E%3Cline%20x1='827'%20y1='14'%20x2='827'%20y2='788'%20stroke='%23475569'%20stroke-width='4'/%3E%3Ccircle%20cx='827'%20cy='401'%20r='108'%20fill='none'%20stroke='%23334155'%20stroke-width='4'/%3E%3Ctext%20x='32'%20y='64'%20fill='%2394a3b8'%20font-size='42'%20font-family='monospace'%3EARCP%20Field%3C/text%3E%3C/svg%3E";

#[derive(Clone, Debug)]
struct TunableState {
    bool_value: bool,
    i64_value: i64,
    f64_value: f64,
    str_value: String,
    bool_array: Vec<bool>,
    i64_array: Vec<i64>,
    f64_array: Vec<f64>,
    str_array: Vec<String>,
    selected_mode: String,
    drive_pid: [f64; 7],
    arm_pid: [f64; 8],
}

impl Default for TunableState {
    fn default() -> Self {
        Self {
            bool_value: true,
            i64_value: 42,
            f64_value: 0.35,
            str_value: "ready".to_string(),
            bool_array: vec![true, false, true, true],
            i64_array: vec![10, 20, 30],
            f64_array: vec![0.20, 0.40, 0.60, 0.80],
            str_array: vec!["alpha".to_string(), "beta".to_string(), "gamma".to_string()],
            selected_mode: MODE_OPTIONS[0].to_string(),
            drive_pid: [0.180, 0.000, 0.008, 0.120, 2.260, 0.140, 3.500],
            arm_pid: [0.390, 0.000, 0.015, 0.540, 0.110, 0.860, 0.040, 1.260],
        }
    }
}

impl TunableState {
    fn apply(&mut self, signal_id: u16, value: SignalValue) -> bool {
        match (signal_id, value) {
            (SIG_TUNABLE_BOOL, SignalValue::Bool(next)) => {
                self.bool_value = next;
                true
            }
            (SIG_TUNABLE_I64, SignalValue::I64(next)) => {
                self.i64_value = next;
                true
            }
            (SIG_TUNABLE_F64, SignalValue::F64(next)) => {
                self.f64_value = next;
                true
            }
            (SIG_TUNABLE_STR, SignalValue::Str(next)) => {
                self.str_value = next;
                true
            }
            (SIG_TUNABLE_BOOL_ARRAY, SignalValue::BoolArray(next)) => {
                self.bool_array = next;
                true
            }
            (SIG_TUNABLE_I64_ARRAY, SignalValue::I64Array(next)) => {
                self.i64_array = next;
                true
            }
            (SIG_TUNABLE_F64_ARRAY, SignalValue::F64Array(next)) => {
                self.f64_array = next;
                true
            }
            (SIG_TUNABLE_STR_ARRAY, SignalValue::StrArray(next)) => {
                self.str_array = next;
                true
            }
            (SIG_SELECTED_MODE, SignalValue::Str(next)) => {
                self.selected_mode = normalize_mode(&next);
                true
            }
            (SIG_PID_DRIVE_KP, SignalValue::F64(next)) => {
                self.drive_pid[0] = next;
                true
            }
            (SIG_PID_DRIVE_KI, SignalValue::F64(next)) => {
                self.drive_pid[1] = next;
                true
            }
            (SIG_PID_DRIVE_KD, SignalValue::F64(next)) => {
                self.drive_pid[2] = next;
                true
            }
            (SIG_PID_DRIVE_KS, SignalValue::F64(next)) => {
                self.drive_pid[3] = next;
                true
            }
            (SIG_PID_DRIVE_KV, SignalValue::F64(next)) => {
                self.drive_pid[4] = next;
                true
            }
            (SIG_PID_DRIVE_KA, SignalValue::F64(next)) => {
                self.drive_pid[5] = next;
                true
            }
            (SIG_PID_DRIVE_SETPOINT, SignalValue::F64(next)) => {
                self.drive_pid[6] = next;
                true
            }
            (SIG_PID_ARM_KP, SignalValue::F64(next)) => {
                self.arm_pid[0] = next;
                true
            }
            (SIG_PID_ARM_KI, SignalValue::F64(next)) => {
                self.arm_pid[1] = next;
                true
            }
            (SIG_PID_ARM_KD, SignalValue::F64(next)) => {
                self.arm_pid[2] = next;
                true
            }
            (SIG_PID_ARM_KG, SignalValue::F64(next)) => {
                self.arm_pid[3] = next;
                true
            }
            (SIG_PID_ARM_KS, SignalValue::F64(next)) => {
                self.arm_pid[4] = next;
                true
            }
            (SIG_PID_ARM_KV, SignalValue::F64(next)) => {
                self.arm_pid[5] = next;
                true
            }
            (SIG_PID_ARM_KA, SignalValue::F64(next)) => {
                self.arm_pid[6] = next;
                true
            }
            (SIG_PID_ARM_SETPOINT, SignalValue::F64(next)) => {
                self.arm_pid[7] = next;
                true
            }
            _ => false,
        }
    }
}

fn main() {
    let server = ArcpServer::new(ArcpServerConfig::default());
    register_signals(&server);
    seed_server_layouts(&server);
    server.start().expect("server start");

    println!(
        "ARCP sim_dashboard running. control={} realtime={}",
        server.control_port(),
        server.realtime_port()
    );
    println!("Run host dashboard on your computer:");
    println!("  cd athena-arcp/host-dashboard && bun run tauri dev");
    println!("  host=127.0.0.1 control_port={}", server.control_port());
    println!(
        "  settings -> server layout profile '{}' -> Load",
        LAYOUT_PROFILE_SHOWCASE
    );

    let mut tick = 0_u64;
    let mut phase = 0.0_f64;
    let mut enabled = true;
    let mut mode_index = 0_usize;
    let mut tunables = TunableState::default();
    let mut next_on_change_publish = Instant::now();

    loop {
        for event in poll_events(&server) {
            match event {
                RuntimeEvent::TunableSet { signal_id, value } => {
                    if tunables.apply(signal_id, value.clone()) {
                        println!("tunable updated: id={signal_id} value={value:?}");
                    }
                }
                RuntimeEvent::Action { signal_id } if signal_id == SIG_CMD_RESET => {
                    tick = 0;
                    phase = 0.0;
                    mode_index = 0;
                    enabled = true;
                    tunables = TunableState::default();
                    println!("command reset received");
                }
                RuntimeEvent::Action { signal_id } if signal_id == SIG_CMD_TOGGLE_ENABLED => {
                    enabled = !enabled;
                    println!("command toggle_enabled -> {enabled}");
                }
                RuntimeEvent::Action { signal_id } if signal_id == SIG_CMD_CYCLE_MODE => {
                    mode_index = (mode_index + 1) % MODE_OPTIONS.len();
                    tunables.selected_mode = MODE_OPTIONS[mode_index].to_string();
                    println!("command cycle_mode -> {}", tunables.selected_mode);
                }
                _ => {}
            }
        }

        tick = tick.saturating_add(1);
        phase += 0.045;
        if phase > std::f64::consts::TAU {
            phase -= std::f64::consts::TAU;
        }

        publish_high_rate(&server, tick, phase, enabled, mode_index, &tunables);
        publish_tunables(&server, &tunables);
        publish_aux_signals(&server, tick, phase, enabled, mode_index, &tunables);

        if next_on_change_publish.elapsed() >= Duration::from_millis(250) {
            publish_on_change(&server, tick, phase, enabled, mode_index, &tunables);
            next_on_change_publish = Instant::now();
        }

        thread::sleep(Duration::from_millis(20));
    }
}

fn seed_server_layouts(server: &ArcpServer) {
    server
        .store_layout(
            LAYOUT_PROFILE_SHOWCASE,
            include_str!("layouts/sim_all_layouts.json"),
        )
        .expect("store showcase layout");
}

fn register_signals(server: &ArcpServer) {
    for descriptor in descriptor_catalog() {
        server.register_signal(descriptor).expect("register signal");
    }
}

fn descriptor_catalog() -> Vec<SignalDescriptor> {
    let mut descriptors = vec![
        SignalDescriptor::telemetry(
            SIG_HR_BOOL,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Example/high_rate/bool",
        ),
        SignalDescriptor::telemetry(
            SIG_HR_I64,
            SignalType::I64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Example/high_rate/i64",
        ),
        SignalDescriptor::telemetry(
            SIG_HR_F64,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Example/high_rate/f64",
        ),
        SignalDescriptor::telemetry(
            SIG_HR_STR,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Example/high_rate/string",
        ),
        SignalDescriptor::telemetry(
            SIG_HR_BOOL_ARRAY,
            SignalType::BoolArray,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Example/high_rate/bool_array",
        ),
        SignalDescriptor::telemetry(
            SIG_HR_I64_ARRAY,
            SignalType::I64Array,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Example/high_rate/i64_array",
        ),
        SignalDescriptor::telemetry(
            SIG_HR_F64_ARRAY,
            SignalType::F64Array,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Example/high_rate/f64_array",
        ),
        SignalDescriptor::telemetry(
            SIG_HR_STR_ARRAY,
            SignalType::StrArray,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Volatile,
            "/Athena/Example/high_rate/string_array",
        ),
        SignalDescriptor::telemetry(
            SIG_OC_BOOL,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Example/on_change/bool",
        ),
        SignalDescriptor::telemetry(
            SIG_OC_I64,
            SignalType::I64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Example/on_change/i64",
        ),
        SignalDescriptor::telemetry(
            SIG_OC_F64,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Example/on_change/f64",
        ),
        SignalDescriptor::telemetry(
            SIG_OC_STR,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Example/on_change/string",
        ),
        SignalDescriptor::telemetry(
            SIG_OC_BOOL_ARRAY,
            SignalType::BoolArray,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Example/on_change/bool_array",
        ),
        SignalDescriptor::telemetry(
            SIG_OC_I64_ARRAY,
            SignalType::I64Array,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Example/on_change/i64_array",
        ),
        SignalDescriptor::telemetry(
            SIG_OC_F64_ARRAY,
            SignalType::F64Array,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Example/on_change/f64_array",
        ),
        SignalDescriptor::telemetry(
            SIG_OC_STR_ARRAY,
            SignalType::StrArray,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Example/on_change/string_array",
        ),
        SignalDescriptor::telemetry(
            SIG_TUNABLE_BOOL,
            SignalType::Bool,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Example/tunable/bool",
        ),
        SignalDescriptor::telemetry(
            SIG_TUNABLE_I64,
            SignalType::I64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Example/tunable/i64",
        ),
        SignalDescriptor::telemetry(
            SIG_TUNABLE_F64,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Example/tunable/f64",
        ),
        SignalDescriptor::telemetry(
            SIG_TUNABLE_STR,
            SignalType::Str,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Example/tunable/string",
        ),
        SignalDescriptor::telemetry(
            SIG_TUNABLE_BOOL_ARRAY,
            SignalType::BoolArray,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Example/tunable/bool_array",
        ),
        SignalDescriptor::telemetry(
            SIG_TUNABLE_I64_ARRAY,
            SignalType::I64Array,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Example/tunable/i64_array",
        ),
        SignalDescriptor::telemetry(
            SIG_TUNABLE_F64_ARRAY,
            SignalType::F64Array,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Example/tunable/f64_array",
        ),
        SignalDescriptor::telemetry(
            SIG_TUNABLE_STR_ARRAY,
            SignalType::StrArray,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Example/tunable/string_array",
        ),
        SignalDescriptor::command(
            SIG_CMD_RESET,
            SignalType::Bool,
            "/Athena/Example/command/reset",
        ),
        SignalDescriptor::command(
            SIG_CMD_TOGGLE_ENABLED,
            SignalType::Bool,
            "/Athena/Example/command/toggle_enabled",
        ),
        SignalDescriptor::telemetry(
            SIG_MATCH_TIME,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Robot/match_time_s",
        ),
        SignalDescriptor::telemetry(
            SIG_BATTERY_VOLTAGE,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Robot/battery_voltage",
        ),
        SignalDescriptor::telemetry(
            SIG_ROBOT_POSE,
            SignalType::F64Array,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Robot/pose_xy_deg",
        ),
        SignalDescriptor::telemetry(
            SIG_TRAJECTORY_POINTS,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Robot/trajectory_xy",
        ),
        SignalDescriptor::telemetry(
            SIG_SUBSYSTEM_HEALTH,
            SignalType::BoolArray,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Robot/subsystem_health",
        ),
        SignalDescriptor::telemetry(
            SIG_CAMERA_OVERLAY,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Vision/overlay_json",
        ),
        SignalDescriptor::telemetry(
            SIG_ROBOT_X,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Robot/pose_x_m",
        ),
        SignalDescriptor::telemetry(
            SIG_ROBOT_Y,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Robot/pose_y_m",
        ),
        SignalDescriptor::telemetry(
            SIG_ROBOT_HEADING_DEG,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Robot/heading_deg",
        ),
        SignalDescriptor::telemetry(
            SIG_ROBOT_STATE,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Robot/state",
        ),
        SignalDescriptor::telemetry(
            SIG_GAME_STATE,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Robot/game_state",
        ),
        SignalDescriptor::telemetry(
            SIG_CAMERA_STREAM_URL,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Vision/stream_url",
        ),
        SignalDescriptor::telemetry(
            SIG_CAMERA_TARGETS,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Vision/targets",
        ),
        SignalDescriptor::telemetry(
            SIG_CAMERA_DETECTIONS,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Vision/detections",
        ),
        SignalDescriptor::telemetry(
            SIG_GRAPH_CHANNEL_A,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Graph/channel_a",
        ),
        SignalDescriptor::telemetry(
            SIG_GRAPH_CHANNEL_B,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Graph/channel_b",
        ),
        SignalDescriptor::telemetry(
            SIG_GRAPH_CHANNEL_C,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Graph/channel_c",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_DRIVE_KP,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/drive_pid/kp",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_DRIVE_KI,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/drive_pid/ki",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_DRIVE_KD,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/drive_pid/kd",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_DRIVE_KS,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/drive_pid/ks",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_DRIVE_KV,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/drive_pid/kv",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_DRIVE_KA,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/drive_pid/ka",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_DRIVE_SETPOINT,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/drive_pid/setpoint",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_ARM_KP,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/arm_pid/kp",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_ARM_KI,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/arm_pid/ki",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_ARM_KD,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/arm_pid/kd",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_ARM_KG,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/arm_pid/kg",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_ARM_KS,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/arm_pid/ks",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_ARM_KV,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/arm_pid/kv",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_ARM_KA,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/arm_pid/ka",
        ),
        SignalDescriptor::telemetry(
            SIG_PID_ARM_SETPOINT,
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Control/arm_pid/setpoint",
        ),
        SignalDescriptor::telemetry(
            SIG_FIELD_IMAGE_URL,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Field/image_url",
        ),
        SignalDescriptor::telemetry(
            SIG_MECH2D_JSON,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Mechanism/mech2d_json",
        ),
        SignalDescriptor::telemetry(
            SIG_SWERVE_MODULES,
            SignalType::F64Array,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/swerve/modules",
        ),
        SignalDescriptor::telemetry(
            SIG_SWERVE_FL_ANGLE_DEG,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/swerve/fl/angle_deg",
        ),
        SignalDescriptor::telemetry(
            SIG_SWERVE_FL_SPEED_MPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/swerve/fl/speed_mps",
        ),
        SignalDescriptor::telemetry(
            SIG_SWERVE_FR_ANGLE_DEG,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/swerve/fr/angle_deg",
        ),
        SignalDescriptor::telemetry(
            SIG_SWERVE_FR_SPEED_MPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/swerve/fr/speed_mps",
        ),
        SignalDescriptor::telemetry(
            SIG_SWERVE_BL_ANGLE_DEG,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/swerve/bl/angle_deg",
        ),
        SignalDescriptor::telemetry(
            SIG_SWERVE_BL_SPEED_MPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/swerve/bl/speed_mps",
        ),
        SignalDescriptor::telemetry(
            SIG_SWERVE_BR_ANGLE_DEG,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/swerve/br/angle_deg",
        ),
        SignalDescriptor::telemetry(
            SIG_SWERVE_BR_SPEED_MPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/swerve/br/speed_mps",
        ),
        SignalDescriptor::telemetry(
            SIG_DIFF_LEFT_SPEED_MPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/diff/left_speed_mps",
        ),
        SignalDescriptor::telemetry(
            SIG_DIFF_RIGHT_SPEED_MPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/diff/right_speed_mps",
        ),
        SignalDescriptor::telemetry(
            SIG_DIFF_HEADING_DEG,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/diff/heading_deg",
        ),
        SignalDescriptor::telemetry(
            SIG_MODE_OPTIONS,
            SignalType::StrArray,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Mode/options",
        ),
        SignalDescriptor::telemetry(
            SIG_SELECTED_MODE,
            SignalType::Str,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            "/Athena/Mode/selected",
        ),
        SignalDescriptor::command(
            SIG_CMD_CYCLE_MODE,
            SignalType::Bool,
            "/Athena/Example/command/cycle_mode",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_STATUS_IS_SIMULATION,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Performance/Status/isSimulation",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_STATUS_IS_DS_ATTACHED,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Performance/Status/isDSAttached",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_STATUS_IS_FMS_ATTACHED,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Performance/Status/isFMSAttached",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_LOOP_PERIOD_SEC,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Performance/Loop/periodSec",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_LOOP_TOTAL_MS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Performance/Loop/totalMs",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_LOOP_UTILIZATION_PERCENT,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Performance/Loop/utilizationPercent",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_POWER_BATTERY_VOLTAGE,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Performance/Power/batteryVoltage",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_POWER_INPUT_CURRENT_AMPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Performance/Power/inputCurrentAmps",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_POWER_CPU_TEMP_CELSIUS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Performance/Power/cpuTempCelsius",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_MEMORY_HEAP_USED_BYTES,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Performance/Memory/heapUsedBytes",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_PERF_MEMORY_HEAP_COMMITTED_BYTES,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Performance/Memory/heapCommittedBytes",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_AUTO_ROUTINE_COUNT,
            SignalType::I64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Auto/routineCount",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_AUTO_FOLLOWER_ENABLED,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Auto/Follower/enabled",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_AUTO_SELECTED_ID,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Auto/Selected/id",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_AUTO_SELECTED_DISPLAY_NAME,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Auto/Selected/displayName",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_AUTO_HAS_TRAJECTORY,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Auto/hasTrajectory",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_LOCALIZATION_HEALTH_POSE_JUMP_METERS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Localization/Health/poseJumpMeters",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_LOCALIZATION_HEALTH_DRIFT_RATE_MPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Localization/Health/driftRateMetersPerSec",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_LOCALIZATION_HEALTH_VISION_ACCEPT_RATE,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Localization/Health/visionAcceptRate",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_LOCALIZATION_HEALTH_SLIP_ACTIVE,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Localization/Health/slipActive",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_LOCALIZATION_HEALTH_SLIP_MODE,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Localization/Health/slipMode",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_VISION_CAMERA_COUNT,
            SignalType::I64,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Vision/cameraCount",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_VISION_HAS_LOCALIZATION,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Vision/hasLocalization",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_CONFIG_BASE_URL,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Config/baseUrl",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_CONFIG_DIAGNOSTICS_URL,
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Config/diagnosticsUrl",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_DRIVETRAIN_ROBOTSPEEDS_DRIVER_VX_MPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/RobotSpeeds/driver/vxMps",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_DRIVETRAIN_ROBOTSPEEDS_DRIVER_VY_MPS,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/RobotSpeeds/driver/vyMps",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_DRIVETRAIN_ROBOTSPEEDS_DRIVER_OMEGA_RAD_PER_SEC,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/RobotSpeeds/driver/omegaRadPerSec",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_DRIVETRAIN_DRIFT_CORRECTION_ENABLED,
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            "/Athena/Drivetrain/DriftCorrection/enabled",
        ),
        SignalDescriptor::telemetry(
            SIG_NT4_DRIVETRAIN_DRIFT_CORRECTION_DESIRED_HEADING_DEG,
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            "/Athena/Drivetrain/DriftCorrection/desiredHeadingDeg",
        ),
    ];

    // Demonstrate host-scoped persistence override on one tunable.
    if let Some(selected_mode) = descriptors
        .iter_mut()
        .find(|descriptor| descriptor.signal_id == SIG_SELECTED_MODE)
    {
        selected_mode.persistence_scope = PersistenceScope::HostWorkspace;
        selected_mode.persistence_key = Some(String::from("dashboard.selected_mode"));
        selected_mode.metadata_hash = selected_mode.metadata_fingerprint();
    }

    descriptors
}

fn publish_high_rate(
    server: &ArcpServer,
    tick: u64,
    phase: f64,
    enabled: bool,
    mode_index: usize,
    tunables: &TunableState,
) {
    let mode = MODE_OPTIONS[mode_index];
    let wheel_base = phase.sin() * (3.0 + tunables.f64_value.max(0.05));
    let wheel_speeds = vec![
        wheel_base * 0.94,
        wheel_base * 1.01,
        wheel_base * 0.98,
        wheel_base * 1.05,
    ];

    let _ = server.publish_bool(SIG_HR_BOOL, enabled);
    let _ = server.publish_i64(SIG_HR_I64, (phase.sin() * 1000.0) as i64);
    let _ = server.publish_f64(SIG_HR_F64, wheel_base);
    let _ = server.publish_string(SIG_HR_STR, format!("mode={mode} tick={tick}"));
    let _ = server.publish_bool_array(
        SIG_HR_BOOL_ARRAY,
        vec![enabled, phase.sin().is_sign_positive(), mode_index % 2 == 0],
    );
    let _ = server.publish_i64_array(
        SIG_HR_I64_ARRAY,
        vec![
            tick as i64,
            tick.saturating_add(1) as i64,
            (phase.cos() * 250.0) as i64,
        ],
    );
    let _ = server.publish_f64_array(SIG_HR_F64_ARRAY, wheel_speeds);
    let _ = server.publish_string_array(
        SIG_HR_STR_ARRAY,
        vec![
            format!("mode={mode}"),
            format!("enabled={enabled}"),
            format!("tick_mod_100={}", tick % 100),
        ],
    );

    let elapsed_s = tick as f64 * 0.02;
    let saw = (tick % 140) as f64 / 140.0;
    let graph_a = (elapsed_s * 2.2).sin() * 3.1;
    let graph_b = (elapsed_s * 1.35).cos() * 2.7 + (elapsed_s * 0.28).sin() * 0.65;
    let graph_c = (saw * 2.0 - 1.0) * 3.0 + (elapsed_s * 0.95).sin() * 0.28;
    let _ = server.publish_f64(SIG_GRAPH_CHANNEL_A, graph_a);
    let _ = server.publish_f64(SIG_GRAPH_CHANNEL_B, graph_b);
    let _ = server.publish_f64(SIG_GRAPH_CHANNEL_C, graph_c);
}

fn publish_on_change(
    server: &ArcpServer,
    tick: u64,
    phase: f64,
    enabled: bool,
    mode_index: usize,
    tunables: &TunableState,
) {
    let _ = server.publish_bool(SIG_OC_BOOL, mode_index == 0 || !enabled);
    let _ = server.publish_i64(SIG_OC_I64, (tick / 15) as i64);
    let _ = server.publish_f64(SIG_OC_F64, 11.5 + phase.cos() * 0.9);
    let _ = server.publish_string(
        SIG_OC_STR,
        format!(
            "mode={} selected={}",
            MODE_OPTIONS[mode_index], tunables.selected_mode
        ),
    );
    let _ = server.publish_bool_array(
        SIG_OC_BOOL_ARRAY,
        vec![
            enabled,
            tunables.bool_value,
            tunables
                .bool_array
                .iter()
                .copied()
                .reduce(|acc, value| acc && value)
                .unwrap_or(false),
        ],
    );
    let _ = server.publish_i64_array(
        SIG_OC_I64_ARRAY,
        vec![
            tunables.i64_value,
            tunables.i64_array.iter().sum(),
            (phase * 100.0) as i64,
        ],
    );
    let _ = server.publish_f64_array(
        SIG_OC_F64_ARRAY,
        vec![
            tunables.f64_value,
            tunables.f64_array.iter().copied().sum::<f64>(),
            phase.sin() * 100.0,
        ],
    );
    let _ = server.publish_string_array(
        SIG_OC_STR_ARRAY,
        vec![
            format!("selected={}", tunables.selected_mode),
            format!("count={}", tunables.str_array.len()),
            "on_change_snapshot".to_string(),
        ],
    );
}

fn publish_tunables(server: &ArcpServer, tunables: &TunableState) {
    let _ = server.publish_bool(SIG_TUNABLE_BOOL, tunables.bool_value);
    let _ = server.publish_i64(SIG_TUNABLE_I64, tunables.i64_value);
    let _ = server.publish_f64(SIG_TUNABLE_F64, tunables.f64_value);
    let _ = server.publish_string(SIG_TUNABLE_STR, tunables.str_value.clone());
    let _ = server.publish_bool_array(SIG_TUNABLE_BOOL_ARRAY, tunables.bool_array.clone());
    let _ = server.publish_i64_array(SIG_TUNABLE_I64_ARRAY, tunables.i64_array.clone());
    let _ = server.publish_f64_array(SIG_TUNABLE_F64_ARRAY, tunables.f64_array.clone());
    let _ = server.publish_string_array(SIG_TUNABLE_STR_ARRAY, tunables.str_array.clone());
    let _ = server.publish_string(SIG_SELECTED_MODE, tunables.selected_mode.clone());
    let _ = server.publish_f64(SIG_PID_DRIVE_KP, tunables.drive_pid[0]);
    let _ = server.publish_f64(SIG_PID_DRIVE_KI, tunables.drive_pid[1]);
    let _ = server.publish_f64(SIG_PID_DRIVE_KD, tunables.drive_pid[2]);
    let _ = server.publish_f64(SIG_PID_DRIVE_KS, tunables.drive_pid[3]);
    let _ = server.publish_f64(SIG_PID_DRIVE_KV, tunables.drive_pid[4]);
    let _ = server.publish_f64(SIG_PID_DRIVE_KA, tunables.drive_pid[5]);
    let _ = server.publish_f64(SIG_PID_DRIVE_SETPOINT, tunables.drive_pid[6]);
    let _ = server.publish_f64(SIG_PID_ARM_KP, tunables.arm_pid[0]);
    let _ = server.publish_f64(SIG_PID_ARM_KI, tunables.arm_pid[1]);
    let _ = server.publish_f64(SIG_PID_ARM_KD, tunables.arm_pid[2]);
    let _ = server.publish_f64(SIG_PID_ARM_KG, tunables.arm_pid[3]);
    let _ = server.publish_f64(SIG_PID_ARM_KS, tunables.arm_pid[4]);
    let _ = server.publish_f64(SIG_PID_ARM_KV, tunables.arm_pid[5]);
    let _ = server.publish_f64(SIG_PID_ARM_KA, tunables.arm_pid[6]);
    let _ = server.publish_f64(SIG_PID_ARM_SETPOINT, tunables.arm_pid[7]);
}

fn publish_aux_signals(
    server: &ArcpServer,
    tick: u64,
    phase: f64,
    enabled: bool,
    mode_index: usize,
    tunables: &TunableState,
) {
    let elapsed_s = tick as f64 * 0.02;
    let elapsed_cycle = elapsed_s % 150.0;
    let match_time_remaining = (150.0 - elapsed_cycle).max(0.0);
    let discharge = 12.9 - (elapsed_cycle / 150.0) * 2.7;
    let load_sag = ((elapsed_s * 1.8).sin().abs().powf(1.45)) * 1.85;
    let battery_voltage = (discharge - load_sag + (elapsed_s * 0.22).cos() * 0.18).clamp(8.4, 13.0);
    let pose_x = (8.27 + (elapsed_s * 0.43).sin() * 7.95).clamp(0.05, 16.49);
    let pose_y = (4.01 + (elapsed_s * 0.31 + 0.9).sin() * 3.75).clamp(0.05, 7.97);
    let heading_deg = (elapsed_s * 57.0).rem_euclid(360.0);
    let robot_state = robot_state_for_tick(tick);
    let game_state = game_state_for_elapsed(elapsed_cycle);

    let _ = server.publish_f64(SIG_MATCH_TIME, elapsed_cycle);
    let _ = server.publish_f64(SIG_BATTERY_VOLTAGE, battery_voltage);
    let _ = server.publish_f64_array(SIG_ROBOT_POSE, vec![pose_x, pose_y, heading_deg]);
    let _ = server.publish_f64(SIG_ROBOT_X, pose_x);
    let _ = server.publish_f64(SIG_ROBOT_Y, pose_y);
    let _ = server.publish_f64(SIG_ROBOT_HEADING_DEG, heading_deg);
    let _ = server.publish_string(SIG_ROBOT_STATE, robot_state.to_string());
    let _ = server.publish_string(SIG_GAME_STATE, game_state.to_string());
    let _ = server.publish_string(SIG_TRAJECTORY_POINTS, trajectory_points_json(elapsed_s));
    let _ = server.publish_bool_array(
        SIG_SUBSYSTEM_HEALTH,
        vec![
            tunables.bool_value,
            battery_voltage > 9.3,
            mode_index > 0,
            match_time_remaining > 15.0,
            tunables.selected_mode != "disabled",
            !tunables.str_value.is_empty(),
        ],
    );
    let _ = server.publish_string(
        SIG_CAMERA_OVERLAY,
        format!(
            "{{\"state\":\"{robot_state}\",\"game\":\"{game_state}\",\"pose\":[{pose_x:.2},{pose_y:.2},{heading_deg:.1}],\"targets\":[{{\"id\":3,\"tx\":{:.2},\"ty\":{:.2}}}]}}",
            phase.sin() * 12.0,
            phase.cos() * 6.0
        ),
    );
    let _ = server.publish_string(SIG_CAMERA_STREAM_URL, CAMERA_STREAM_DATA_URI.to_string());
    let _ = server.publish_string(SIG_CAMERA_TARGETS, camera_targets_json(elapsed_s));
    let _ = server.publish_string(SIG_CAMERA_DETECTIONS, camera_detections_json(elapsed_s));
    let _ = server.publish_string(SIG_FIELD_IMAGE_URL, FIELD_IMAGE_DATA_URI.to_string());

    let swerve_fl_angle = (elapsed_s * 81.0).rem_euclid(360.0);
    let swerve_fr_angle = (elapsed_s * 73.0 + 35.0).rem_euclid(360.0);
    let swerve_bl_angle = (elapsed_s * 65.0 + 92.0).rem_euclid(360.0);
    let swerve_br_angle = (elapsed_s * 70.0 + 140.0).rem_euclid(360.0);
    let swerve_fl_speed = (elapsed_s * 1.20).sin() * 4.6;
    let swerve_fr_speed = (elapsed_s * 1.37 + 0.4).sin() * 4.3;
    let swerve_bl_speed = (elapsed_s * 1.07 + 0.8).sin() * 4.8;
    let swerve_br_speed = (elapsed_s * 1.16 + 1.1).sin() * 4.5;
    let _ = server.publish_f64(SIG_SWERVE_FL_ANGLE_DEG, swerve_fl_angle);
    let _ = server.publish_f64(SIG_SWERVE_FR_ANGLE_DEG, swerve_fr_angle);
    let _ = server.publish_f64(SIG_SWERVE_BL_ANGLE_DEG, swerve_bl_angle);
    let _ = server.publish_f64(SIG_SWERVE_BR_ANGLE_DEG, swerve_br_angle);
    let _ = server.publish_f64(SIG_SWERVE_FL_SPEED_MPS, swerve_fl_speed);
    let _ = server.publish_f64(SIG_SWERVE_FR_SPEED_MPS, swerve_fr_speed);
    let _ = server.publish_f64(SIG_SWERVE_BL_SPEED_MPS, swerve_bl_speed);
    let _ = server.publish_f64(SIG_SWERVE_BR_SPEED_MPS, swerve_br_speed);
    let _ = server.publish_f64_array(
        SIG_SWERVE_MODULES,
        vec![
            swerve_fl_angle,
            swerve_fl_speed,
            swerve_fr_angle,
            swerve_fr_speed,
            swerve_bl_angle,
            swerve_bl_speed,
            swerve_br_angle,
            swerve_br_speed,
        ],
    );

    let diff_left_speed = (elapsed_s * 0.95).sin() * 3.8;
    let diff_right_speed = (elapsed_s * 0.95 + 0.62).sin() * 3.8;
    let diff_heading = (elapsed_s * 42.0).rem_euclid(360.0);
    let _ = server.publish_f64(SIG_DIFF_LEFT_SPEED_MPS, diff_left_speed);
    let _ = server.publish_f64(SIG_DIFF_RIGHT_SPEED_MPS, diff_right_speed);
    let _ = server.publish_f64(SIG_DIFF_HEADING_DEG, diff_heading);

    let _ = server.publish_string(SIG_MECH2D_JSON, mech2d_json(elapsed_s));
    let _ = server.publish_string_array(
        SIG_MODE_OPTIONS,
        MODE_OPTIONS
            .iter()
            .map(|value| (*value).to_string())
            .collect(),
    );

    let loop_period_sec = 0.02;
    let loop_total_ms = 4.8 + (elapsed_s * 0.91).sin().abs() * 3.3;
    let loop_utilization_percent =
        (loop_total_ms / (loop_period_sec * 1000.0) * 100.0).clamp(0.0, 100.0);
    let input_current_amps = (15.0 + (elapsed_s * 1.7).sin().abs() * 70.0).clamp(8.0, 120.0);
    let cpu_temp_celsius = 46.0 + (elapsed_s * 0.19).sin().abs() * 18.0;
    let heap_used_bytes = 42_000_000.0 + (elapsed_s * 0.13).sin().abs() * 19_000_000.0;
    let heap_committed_bytes = heap_used_bytes + 26_000_000.0;
    let vision_accept_rate = (0.75 + (elapsed_s * 0.37).sin() * 0.18).clamp(0.25, 0.99);
    let pose_jump_meters = (elapsed_s * 0.81).sin().abs() * 0.18;
    let drift_rate_mps = (elapsed_s * 0.62).sin().abs() * 0.65;
    let slip_active = drift_rate_mps > 0.46;
    let slip_mode = if slip_active { "aggressive" } else { "normal" };
    let driver_vx_mps = (elapsed_s * 0.73).sin() * 4.1;
    let driver_vy_mps = (elapsed_s * 0.67 + 0.4).sin() * 3.1;
    let driver_omega_rad_per_sec = (elapsed_s * 0.88 + 0.7).sin() * 3.7;
    let drift_desired_heading_deg =
        (heading_deg + (elapsed_s * 0.43).sin() * 22.0).rem_euclid(360.0);
    let fms_attached = matches!(game_state, "auto" | "tele" | "endgame");
    let selected_auto_id = format!("{}_routine", tunables.selected_mode);
    let selected_auto_name = match tunables.selected_mode.as_str() {
        "auto" => "Center 3-piece",
        "tele" => "Driver Controlled",
        "test" => "Characterization",
        _ => "Idle",
    };

    let _ = server.publish_bool(SIG_NT4_PERF_STATUS_IS_SIMULATION, true);
    let _ = server.publish_bool(SIG_NT4_PERF_STATUS_IS_DS_ATTACHED, true);
    let _ = server.publish_bool(SIG_NT4_PERF_STATUS_IS_FMS_ATTACHED, fms_attached);
    let _ = server.publish_f64(SIG_NT4_PERF_LOOP_PERIOD_SEC, loop_period_sec);
    let _ = server.publish_f64(SIG_NT4_PERF_LOOP_TOTAL_MS, loop_total_ms);
    let _ = server.publish_f64(
        SIG_NT4_PERF_LOOP_UTILIZATION_PERCENT,
        loop_utilization_percent,
    );
    let _ = server.publish_f64(SIG_NT4_PERF_POWER_BATTERY_VOLTAGE, battery_voltage);
    let _ = server.publish_f64(SIG_NT4_PERF_POWER_INPUT_CURRENT_AMPS, input_current_amps);
    let _ = server.publish_f64(SIG_NT4_PERF_POWER_CPU_TEMP_CELSIUS, cpu_temp_celsius);
    let _ = server.publish_f64(SIG_NT4_PERF_MEMORY_HEAP_USED_BYTES, heap_used_bytes);
    let _ = server.publish_f64(
        SIG_NT4_PERF_MEMORY_HEAP_COMMITTED_BYTES,
        heap_committed_bytes,
    );

    let _ = server.publish_i64(SIG_NT4_AUTO_ROUTINE_COUNT, 8);
    let _ = server.publish_bool(SIG_NT4_AUTO_FOLLOWER_ENABLED, mode_index > 0 && enabled);
    let _ = server.publish_string(SIG_NT4_AUTO_SELECTED_ID, selected_auto_id);
    let _ = server.publish_string(
        SIG_NT4_AUTO_SELECTED_DISPLAY_NAME,
        selected_auto_name.to_string(),
    );
    let _ = server.publish_bool(SIG_NT4_AUTO_HAS_TRAJECTORY, true);

    let _ = server.publish_f64(
        SIG_NT4_LOCALIZATION_HEALTH_POSE_JUMP_METERS,
        pose_jump_meters,
    );
    let _ = server.publish_f64(SIG_NT4_LOCALIZATION_HEALTH_DRIFT_RATE_MPS, drift_rate_mps);
    let _ = server.publish_f64(
        SIG_NT4_LOCALIZATION_HEALTH_VISION_ACCEPT_RATE,
        vision_accept_rate,
    );
    let _ = server.publish_bool(SIG_NT4_LOCALIZATION_HEALTH_SLIP_ACTIVE, slip_active);
    let _ = server.publish_string(SIG_NT4_LOCALIZATION_HEALTH_SLIP_MODE, slip_mode.to_string());

    let _ = server.publish_i64(SIG_NT4_VISION_CAMERA_COUNT, 2);
    let _ = server.publish_bool(SIG_NT4_VISION_HAS_LOCALIZATION, true);
    let _ = server.publish_string(SIG_NT4_CONFIG_BASE_URL, "http://127.0.0.1:5809".to_string());
    let _ = server.publish_string(
        SIG_NT4_CONFIG_DIAGNOSTICS_URL,
        "http://127.0.0.1:5809/Athena/diagnostics".to_string(),
    );

    let _ = server.publish_f64(SIG_NT4_DRIVETRAIN_ROBOTSPEEDS_DRIVER_VX_MPS, driver_vx_mps);
    let _ = server.publish_f64(SIG_NT4_DRIVETRAIN_ROBOTSPEEDS_DRIVER_VY_MPS, driver_vy_mps);
    let _ = server.publish_f64(
        SIG_NT4_DRIVETRAIN_ROBOTSPEEDS_DRIVER_OMEGA_RAD_PER_SEC,
        driver_omega_rad_per_sec,
    );
    let _ = server.publish_bool(SIG_NT4_DRIVETRAIN_DRIFT_CORRECTION_ENABLED, enabled);
    let _ = server.publish_f64(
        SIG_NT4_DRIVETRAIN_DRIFT_CORRECTION_DESIRED_HEADING_DEG,
        drift_desired_heading_deg,
    );
}

fn robot_state_for_tick(tick: u64) -> &'static str {
    let idx = ((tick / 250) as usize) % ROBOT_STATE_SEQUENCE.len();
    ROBOT_STATE_SEQUENCE[idx]
}

fn game_state_for_elapsed(elapsed_cycle: f64) -> &'static str {
    if elapsed_cycle < 6.0 {
        "disabled"
    } else if elapsed_cycle < 21.0 {
        "auto"
    } else if elapsed_cycle < 120.0 {
        "tele"
    } else if elapsed_cycle < 140.0 {
        "endgame"
    } else {
        "sim"
    }
}

fn trajectory_points_json(elapsed_s: f64) -> String {
    let mut points = Vec::with_capacity(14);
    for idx in 0..14 {
        let t = idx as f64 / 13.0;
        let x = 0.7 + t * 15.0;
        let y = (4.0 + (elapsed_s * 0.35 + t * 4.1).sin() * 2.7).clamp(0.2, 7.8);
        points.push(format!("{{\"x\":{x:.3},\"y\":{y:.3}}}"));
    }
    format!("[{}]", points.join(","))
}

fn camera_targets_json(elapsed_s: f64) -> String {
    let x1 = 0.35 + (elapsed_s * 0.72).sin() * 0.16;
    let y1 = 0.38 + (elapsed_s * 0.64).cos() * 0.13;
    let x2 = 0.62 + (elapsed_s * 0.51).sin() * 0.11;
    let y2 = 0.44 + (elapsed_s * 0.58).cos() * 0.09;
    format!(
        "[{{\"x\":{x1:.3},\"y\":{y1:.3},\"w\":0.160,\"h\":0.210,\"label\":\"tag-3\",\"score\":0.95}},{{\"x\":{x2:.3},\"y\":{y2:.3},\"w\":0.120,\"h\":0.170,\"label\":\"tag-7\",\"score\":0.88}}]"
    )
}

fn camera_detections_json(elapsed_s: f64) -> String {
    let x = 0.24 + (elapsed_s * 0.39).sin() * 0.08;
    let y = 0.57 + (elapsed_s * 0.44).cos() * 0.07;
    format!(
        "[{{\"x\":{x:.3},\"y\":{y:.3},\"w\":0.210,\"h\":0.250,\"label\":\"note\",\"score\":0.79}}]"
    )
}

fn mech2d_json(elapsed_s: f64) -> String {
    let elevator_height = 0.28 + ((elapsed_s * 0.72).sin() * 0.5 + 0.5) * 0.46;
    let arm_angle_rad = (elapsed_s * 0.94).sin() * 0.9 + 0.5;
    let wrist_angle_rad = (elapsed_s * 1.22).sin() * 0.6;
    let base_x = 0.30;
    let stage_x = base_x;
    let stage_y = elevator_height;
    let arm_len = 0.36;
    let arm_tip_x = stage_x + arm_len * arm_angle_rad.cos();
    let arm_tip_y = stage_y + arm_len * arm_angle_rad.sin();
    let wrist_len = 0.16;
    let wrist_tip_x = arm_tip_x + wrist_len * (arm_angle_rad + wrist_angle_rad).cos();
    let wrist_tip_y = arm_tip_y + wrist_len * (arm_angle_rad + wrist_angle_rad).sin();

    format!(
        "{{\"viewport\":[1.0,1.0],\"segments\":[{{\"x1\":{base_x:.3},\"y1\":0.100,\"x2\":{base_x:.3},\"y2\":{stage_y:.3},\"color\":\"#94a3b8\"}},{{\"x1\":{stage_x:.3},\"y1\":{stage_y:.3},\"x2\":{arm_tip_x:.3},\"y2\":{arm_tip_y:.3},\"color\":\"#f87171\"}},{{\"x1\":{arm_tip_x:.3},\"y1\":{arm_tip_y:.3},\"x2\":{wrist_tip_x:.3},\"y2\":{wrist_tip_y:.3},\"color\":\"#38bdf8\"}}],\"joints\":[{{\"x\":{base_x:.3},\"y\":0.100,\"r\":0.018}},{{\"x\":{stage_x:.3},\"y\":{stage_y:.3},\"r\":0.020}},{{\"x\":{arm_tip_x:.3},\"y\":{arm_tip_y:.3},\"r\":0.016}},{{\"x\":{wrist_tip_x:.3},\"y\":{wrist_tip_y:.3},\"r\":0.015}}]}}"
    )
}

fn normalize_mode(raw: &str) -> String {
    let lowered = raw.trim().to_lowercase();
    MODE_OPTIONS
        .iter()
        .find(|entry| **entry == lowered)
        .map(|entry| (*entry).to_string())
        .unwrap_or_else(|| MODE_OPTIONS[0].to_string())
}

fn poll_events(server: &ArcpServer) -> Vec<RuntimeEvent> {
    let mut buf = [0_u8; 4096];
    let written = server.poll_events_into(&mut buf);
    if written == 0 {
        return Vec::new();
    }

    let mut out = Vec::new();
    let mut index = 0_usize;
    while index + 2 <= written {
        let len = u16::from_le_bytes([buf[index], buf[index + 1]]) as usize;
        index += 2;
        if index + len > written {
            break;
        }
        if let Ok(event) = decode_event(&buf[index..index + len]) {
            out.push(event);
        }
        index += len;
    }
    out
}
