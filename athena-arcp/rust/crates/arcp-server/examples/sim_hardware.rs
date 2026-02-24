use std::thread;
use std::time::{Duration, Instant};

use arcp_core::{
    decode_event, RuntimeEvent, SignalAccess, SignalDescriptor, SignalDurability, SignalPolicy,
    SignalType, SignalValue,
};
use arcp_server::{ArcpServer, ArcpServerConfig};

const LAYOUT_PROFILE_HARDWARE: &str = "sim-hardware";

const MOTOR_NAMES: [&str; 4] = ["front_left", "front_right", "back_left", "back_right"];
const ENCODER_NAMES: [&str; 4] = ["front_left", "front_right", "back_left", "back_right"];

const MOTOR_BASE_ID: u16 = 100;
const MOTOR_STRIDE: u16 = 16;
const MOTOR_OUTPUT_COMMAND_BASE_ID: u16 = 180;
const MOTOR_OUTPUT_COMMAND_STRIDE: u16 = 4;
const ENCODER_BASE_ID: u16 = 300;
const ENCODER_STRIDE: u16 = 12;

const SIG_IMU_ROLL_DEG: u16 = 501;
const SIG_IMU_PITCH_DEG: u16 = 502;
const SIG_IMU_YAW_DEG: u16 = 503;
const SIG_IMU_HEADING_DEG: u16 = 504;
const SIG_IMU_ACCEL_XYZ: u16 = 505;
const SIG_IMU_GYRO_XYZ_DPS: u16 = 506;
const SIG_IMU_CONNECTED: u16 = 507;
const SIG_IMU_INVERTED: u16 = 508;
const SIG_IMU_CAN_ID: u16 = 509;
const SIG_IMU_CANBUS: u16 = 510;
const SIG_IMU_TYPE: u16 = 511;
const SIG_IMU_MAX_LINEAR_SPEED: u16 = 512;
const SIG_IMU_MAX_RADIAL_SPEED: u16 = 513;
const SIG_IMU_MAX_SPEED_WINDOW_SEC: u16 = 514;
const SIG_IMU_ANGULAR_ACCEL_Z_DPS2: u16 = 515;
const SIG_IMU_X_SPEED_RAD_PER_SEC: u16 = 516;
const SIG_IMU_Y_SPEED_RAD_PER_SEC: u16 = 517;
const SIG_IMU_THETA_SPEED_RAD_PER_SEC: u16 = 518;
const SIG_IMU_MOVEMENT_SPEED_RAD_PER_SEC: u16 = 519;
const SIG_IMU_X_SPEED_MPS: u16 = 520;
const SIG_IMU_Y_SPEED_MPS: u16 = 521;
const SIG_IMU_MOVEMENT_SPEED_MPS: u16 = 522;
const SIG_IMU_MOVEMENT_SPEED_NORMALIZED: u16 = 523;
const SIG_IMU_NORMALIZED_SPEED: u16 = 524;
const SIG_IMU_VEL_X_DPS: u16 = 525;
const SIG_IMU_VEL_Y_DPS: u16 = 526;
const SIG_IMU_VEL_Z_DPS: u16 = 527;
const SIG_IMU_MAG_XYZ_UT: u16 = 528;

const SIG_CMD_RESET: u16 = 900;
const SIG_HW_STATUS: u16 = 950;

const MOTOR_FIELD_OUTPUT: u16 = 1;
const MOTOR_FIELD_VELOCITY_RPS: u16 = 2;
const MOTOR_FIELD_POSITION_ROT: u16 = 3;
const MOTOR_FIELD_CURRENT_A: u16 = 4;
const MOTOR_FIELD_TEMPERATURE_C: u16 = 5;
const MOTOR_FIELD_VOLTAGE_V: u16 = 6;
const MOTOR_FIELD_CONNECTED: u16 = 7;
const MOTOR_FIELD_NEUTRAL_MODE: u16 = 8;
const MOTOR_FIELD_COMMAND: u16 = 9;
const MOTOR_FIELD_CAN_ID: u16 = 10;
const MOTOR_FIELD_CANBUS: u16 = 11;
const MOTOR_FIELD_TYPE: u16 = 12;
const MOTOR_FIELD_STALLED: u16 = 13;
const MOTOR_FIELD_CURRENT_LIMIT_A: u16 = 14;
const MOTOR_FIELD_INVERTED: u16 = 15;
const MOTOR_FIELD_BRAKE_MODE: u16 = 16;

const MOTOR_OUTPUT_COMMAND_PERCENT: u16 = 0;
const MOTOR_OUTPUT_COMMAND_VOLTAGE_V: u16 = 1;
const MOTOR_OUTPUT_COMMAND_VELOCITY_RPS: u16 = 2;

const ENCODER_FIELD_POSITION_ROT: u16 = 1;
const ENCODER_FIELD_VELOCITY_RPS: u16 = 2;
const ENCODER_FIELD_ABSOLUTE_ROT: u16 = 3;
const ENCODER_FIELD_CONNECTED: u16 = 4;
const ENCODER_FIELD_GEAR_RATIO: u16 = 5;
const ENCODER_FIELD_OFFSET_ROT: u16 = 6;
const ENCODER_FIELD_CAN_ID: u16 = 7;
const ENCODER_FIELD_CANBUS: u16 = 8;
const ENCODER_FIELD_TYPE: u16 = 9;
const ENCODER_FIELD_INVERTED: u16 = 10;
const ENCODER_FIELD_SUPPORTS_SIMULATION: u16 = 11;
const ENCODER_FIELD_RAW_ABSOLUTE_ROT: u16 = 12;

const MOTOR_CAN_IDS: [i64; MOTOR_NAMES.len()] = [1, 2, 3, 4];
const MOTOR_CANBUS: [&str; MOTOR_NAMES.len()] = ["rio", "rio", "rio", "rio"];
const MOTOR_TYPES: [&str; MOTOR_NAMES.len()] = ["talonfx", "talonfx", "talonfx", "talonfx"];
const MOTOR_COMMAND_MAX_VOLTAGE_V: f64 = 12.0;
const MOTOR_COMMAND_MAX_VELOCITY_RPS: f64 = 90.0;

const ENCODER_CAN_IDS: [i64; ENCODER_NAMES.len()] = [11, 12, 13, 14];
const ENCODER_CANBUS: [&str; ENCODER_NAMES.len()] = ["rio", "rio", "rio", "rio"];
const ENCODER_TYPES: [&str; ENCODER_NAMES.len()] = ["cancoder", "cancoder", "cancoder", "cancoder"];

const IMU_CAN_ID: i64 = 42;
const IMU_CANBUS: &str = "rio";
const IMU_TYPE: &str = "pigeon2";

const fn motor_id(index: usize, field: u16) -> u16 {
    MOTOR_BASE_ID + (index as u16) * MOTOR_STRIDE + field
}

const fn encoder_id(index: usize, field: u16) -> u16 {
    ENCODER_BASE_ID + (index as u16) * ENCODER_STRIDE + field
}

const fn motor_output_command_id(index: usize, field: u16) -> u16 {
    MOTOR_OUTPUT_COMMAND_BASE_ID + (index as u16) * MOTOR_OUTPUT_COMMAND_STRIDE + field
}

#[derive(Clone, Debug)]
struct SimState {
    motor_command: [f64; MOTOR_NAMES.len()],
    motor_can_id: [i64; MOTOR_NAMES.len()],
    motor_canbus: [String; MOTOR_NAMES.len()],
    motor_position: [f64; MOTOR_NAMES.len()],
    motor_temp_c: [f64; MOTOR_NAMES.len()],
    motor_current_limit_a: [f64; MOTOR_NAMES.len()],
    motor_inverted: [bool; MOTOR_NAMES.len()],
    motor_brake_mode: [bool; MOTOR_NAMES.len()],
    encoder_ratio: [f64; ENCODER_NAMES.len()],
    encoder_offset: [f64; ENCODER_NAMES.len()],
    encoder_can_id: [i64; ENCODER_NAMES.len()],
    encoder_canbus: [String; ENCODER_NAMES.len()],
    encoder_inverted: [bool; ENCODER_NAMES.len()],
    imu_inverted: bool,
    imu_can_id: i64,
    imu_canbus: String,
    imu_max_linear_speed: f64,
    imu_max_radial_speed: f64,
    imu_max_speed_window_sec: f64,
}

impl Default for SimState {
    fn default() -> Self {
        Self {
            motor_command: [0.0, 0.0, 0.0, 0.0],
            motor_can_id: MOTOR_CAN_IDS,
            motor_canbus: MOTOR_CANBUS.map(|entry| entry.to_string()),
            motor_position: [0.0, 0.0, 0.0, 0.0],
            motor_temp_c: [28.0, 28.5, 27.9, 28.2],
            motor_current_limit_a: [60.0, 60.0, 60.0, 60.0],
            motor_inverted: [false, true, false, true],
            motor_brake_mode: [true, true, true, true],
            encoder_ratio: [6.75, 6.75, 6.75, 6.75],
            encoder_offset: [0.02, -0.01, 0.03, -0.02],
            encoder_can_id: ENCODER_CAN_IDS,
            encoder_canbus: ENCODER_CANBUS.map(|entry| entry.to_string()),
            encoder_inverted: [false, false, false, false],
            imu_inverted: false,
            imu_can_id: IMU_CAN_ID,
            imu_canbus: IMU_CANBUS.to_string(),
            imu_max_linear_speed: 4.8,
            imu_max_radial_speed: std::f64::consts::PI * 2.0,
            imu_max_speed_window_sec: 0.35,
        }
    }
}

fn main() {
    let server = start_server_with_fallback();

    println!(
        "ARCP sim_hardware running. control={} realtime={}",
        server.control_port(),
        server.realtime_port()
    );
    println!("Run host dashboard on your computer:");
    println!("  cd athena-arcp/host-dashboard && bun run tauri dev");
    println!("  host=127.0.0.1 control_port={}", server.control_port());
    println!(
        "  settings -> server layout profile '{}' -> Load",
        LAYOUT_PROFILE_HARDWARE
    );

    let mut tick = 0_u64;
    let mut phase = 0.0_f64;
    let mut state = SimState::default();
    let mut next_low_rate_publish = Instant::now();

    loop {
        for event in poll_events(&server) {
            match event {
                RuntimeEvent::Action { signal_id } if signal_id == SIG_CMD_RESET => {
                    state = SimState::default();
                    tick = 0;
                    phase = 0.0;
                    println!("hardware reset command received");
                }
                RuntimeEvent::TunableSet { signal_id, value } => {
                    apply_tunable(&mut state, signal_id, value);
                }
                _ => {}
            }
        }

        tick = tick.saturating_add(1);
        phase += 0.045;
        if phase > std::f64::consts::TAU {
            phase -= std::f64::consts::TAU;
        }

        publish_high_rate(&server, tick, phase, &mut state);

        if next_low_rate_publish.elapsed() >= Duration::from_millis(200) {
            publish_low_rate(&server, tick, phase, &state);
            next_low_rate_publish = Instant::now();
        }

        thread::sleep(Duration::from_millis(20));
    }
}

fn start_server_with_fallback() -> ArcpServer {
    let primary = ArcpServer::new(ArcpServerConfig::default());
    register_hardware_signals(&primary);
    seed_layouts(&primary);
    match primary.start() {
        Ok(()) => primary,
        Err(primary_err) => {
            eprintln!(
                "sim_hardware: default ports unavailable ({primary_err}); retrying with ephemeral ports."
            );
            let fallback_cfg = ArcpServerConfig {
                control_port: 0,
                realtime_port: 0,
                max_signals: 1024,
            };
            let fallback = ArcpServer::new(fallback_cfg);
            register_hardware_signals(&fallback);
            seed_layouts(&fallback);
            fallback.start().expect("server start with ephemeral ports");
            fallback
        }
    }
}

fn seed_layouts(server: &ArcpServer) {
    server
        .store_layout(
            LAYOUT_PROFILE_HARDWARE,
            include_str!("layouts/sim_hardware_layout.json"),
        )
        .expect("store hardware layout");
}

fn register_hardware_signals(server: &ArcpServer) {
    for descriptor in hardware_descriptor_catalog() {
        server.register_signal(descriptor).expect("register signal");
    }
}

fn hardware_descriptor_catalog() -> Vec<SignalDescriptor> {
    let mut descriptors = Vec::new();

    for (index, name) in MOTOR_NAMES.iter().enumerate() {
        let root = format!("/Athena/Hardware/Motors/{name}");
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_OUTPUT),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/output"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_VELOCITY_RPS),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/velocity_rps"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_POSITION_ROT),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/position_rot"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_CURRENT_A),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/current_a"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_TEMPERATURE_C),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/temperature_c"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_VOLTAGE_V),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/voltage_v"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_CONNECTED),
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            format!("{root}/connected"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_NEUTRAL_MODE),
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            format!("{root}/neutral_mode"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_COMMAND),
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/command"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_CAN_ID),
            SignalType::I64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/can_id"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_CANBUS),
            SignalType::Str,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/canbus"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_TYPE),
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            format!("{root}/type"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_STALLED),
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            format!("{root}/stalled"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_CURRENT_LIMIT_A),
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/current_limit_a"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_INVERTED),
            SignalType::Bool,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/inverted"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_id(index, MOTOR_FIELD_BRAKE_MODE),
            SignalType::Bool,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/brake_mode"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_output_command_id(index, MOTOR_OUTPUT_COMMAND_PERCENT),
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/output_percent_command"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_output_command_id(index, MOTOR_OUTPUT_COMMAND_VOLTAGE_V),
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/output_voltage_v_command"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            motor_output_command_id(index, MOTOR_OUTPUT_COMMAND_VELOCITY_RPS),
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/velocity_rps_command"),
        ));
    }

    for (index, name) in ENCODER_NAMES.iter().enumerate() {
        let root = format!("/Athena/Hardware/Encoders/{name}");
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_POSITION_ROT),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/position_rot"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_VELOCITY_RPS),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/velocity_rps"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_ABSOLUTE_ROT),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/absolute_rot"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_CONNECTED),
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            format!("{root}/connected"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_GEAR_RATIO),
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/gear_ratio"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_OFFSET_ROT),
            SignalType::F64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/offset_rot"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_CAN_ID),
            SignalType::I64,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/can_id"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_CANBUS),
            SignalType::Str,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/canbus"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_TYPE),
            SignalType::Str,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            format!("{root}/type"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_INVERTED),
            SignalType::Bool,
            SignalAccess::Write,
            SignalPolicy::Sampled,
            SignalDurability::Persistent,
            format!("{root}/inverted"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_SUPPORTS_SIMULATION),
            SignalType::Bool,
            SignalAccess::Observe,
            SignalPolicy::OnChange,
            SignalDurability::Retained,
            format!("{root}/supports_simulation"),
        ));
        descriptors.push(SignalDescriptor::telemetry(
            encoder_id(index, ENCODER_FIELD_RAW_ABSOLUTE_ROT),
            SignalType::F64,
            SignalAccess::Observe,
            SignalPolicy::HighRate,
            SignalDurability::Retained,
            format!("{root}/raw_absolute_rot"),
        ));
    }

    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_ROLL_DEG,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/roll_deg",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_PITCH_DEG,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/pitch_deg",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_YAW_DEG,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/yaw_deg",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_HEADING_DEG,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/heading_deg",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_ACCEL_XYZ,
        SignalType::F64Array,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/accel_xyz",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_GYRO_XYZ_DPS,
        SignalType::F64Array,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/gyro_xyz_dps",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_CONNECTED,
        SignalType::Bool,
        SignalAccess::Observe,
        SignalPolicy::OnChange,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/connected",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_INVERTED,
        SignalType::Bool,
        SignalAccess::Write,
        SignalPolicy::Sampled,
        SignalDurability::Persistent,
        "/Athena/Hardware/Imu/pigeon2/inverted",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_CAN_ID,
        SignalType::I64,
        SignalAccess::Write,
        SignalPolicy::Sampled,
        SignalDurability::Persistent,
        "/Athena/Hardware/Imu/pigeon2/can_id",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_CANBUS,
        SignalType::Str,
        SignalAccess::Write,
        SignalPolicy::Sampled,
        SignalDurability::Persistent,
        "/Athena/Hardware/Imu/pigeon2/canbus",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_TYPE,
        SignalType::Str,
        SignalAccess::Observe,
        SignalPolicy::OnChange,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/type",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_MAX_LINEAR_SPEED,
        SignalType::F64,
        SignalAccess::Write,
        SignalPolicy::Sampled,
        SignalDurability::Persistent,
        "/Athena/Hardware/Imu/pigeon2/max_linear_speed",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_MAX_RADIAL_SPEED,
        SignalType::F64,
        SignalAccess::Write,
        SignalPolicy::Sampled,
        SignalDurability::Persistent,
        "/Athena/Hardware/Imu/pigeon2/max_radial_speed",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_MAX_SPEED_WINDOW_SEC,
        SignalType::F64,
        SignalAccess::Write,
        SignalPolicy::Sampled,
        SignalDurability::Persistent,
        "/Athena/Hardware/Imu/pigeon2/max_speed_window_sec",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_ANGULAR_ACCEL_Z_DPS2,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/angular_accel_z_dps2",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_X_SPEED_RAD_PER_SEC,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/x_speed_rad_per_sec",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_Y_SPEED_RAD_PER_SEC,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/y_speed_rad_per_sec",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_THETA_SPEED_RAD_PER_SEC,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/theta_speed_rad_per_sec",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_MOVEMENT_SPEED_RAD_PER_SEC,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/movement_speed_rad_per_sec",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_X_SPEED_MPS,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/x_speed_mps",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_Y_SPEED_MPS,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/y_speed_mps",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_MOVEMENT_SPEED_MPS,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/movement_speed_mps",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_MOVEMENT_SPEED_NORMALIZED,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/movement_speed_normalized",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_NORMALIZED_SPEED,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/normalized_speed",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_VEL_X_DPS,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/vel_x_deg_per_sec",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_VEL_Y_DPS,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/vel_y_deg_per_sec",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_VEL_Z_DPS,
        SignalType::F64,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/vel_z_deg_per_sec",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_IMU_MAG_XYZ_UT,
        SignalType::F64Array,
        SignalAccess::Observe,
        SignalPolicy::HighRate,
        SignalDurability::Retained,
        "/Athena/Hardware/Imu/pigeon2/mag_xyz_ut",
    ));

    descriptors.push(SignalDescriptor::command(
        SIG_CMD_RESET,
        SignalType::Bool,
        "/Athena/Hardware/Command/reset",
    ));
    descriptors.push(SignalDescriptor::telemetry(
        SIG_HW_STATUS,
        SignalType::Str,
        SignalAccess::Observe,
        SignalPolicy::OnChange,
        SignalDurability::Retained,
        "/Athena/Hardware/status",
    ));

    descriptors
}

fn publish_high_rate(server: &ArcpServer, tick: u64, phase: f64, state: &mut SimState) {
    let mut motor_velocity = [0.0_f64; MOTOR_NAMES.len()];
    let mut motor_connected = [true; MOTOR_NAMES.len()];

    for index in 0..MOTOR_NAMES.len() {
        let command = state.motor_command[index].clamp(-1.0, 1.0);
        let signed_command = if state.motor_inverted[index] {
            -command
        } else {
            command
        };
        let output =
            (signed_command * (0.86 + 0.14 * (phase * 0.9 + index as f64).sin())).clamp(-1.0, 1.0);
        let velocity_rps = output * 78.0 + (phase * 2.2 + index as f64).sin() * 4.5;
        let dt = 0.02;
        state.motor_position[index] += velocity_rps * dt;
        state.motor_temp_c[index] =
            state.motor_temp_c[index] * 0.985 + (30.0 + output.abs() * 55.0) * 0.015;
        let unclamped_current_a =
            2.8 + output.abs() * 60.0 + (phase * 1.7 + index as f64).cos().abs() * 2.5;
        let current_limit = state.motor_current_limit_a[index].clamp(1.0, 200.0);
        let current_a = unclamped_current_a.min(current_limit);
        let voltage_v = 12.4 - output.abs() * 1.7 - current_a * 0.01;
        let connected = ((tick / 500 + index as u64) % 20) != 0;
        let stall_window = command.abs() > 0.7 && velocity_rps.abs() < 6.0;
        let stalled = connected && stall_window;
        let neutral_mode = if state.motor_brake_mode[index] {
            "Brake"
        } else {
            "Coast"
        };

        motor_velocity[index] = velocity_rps;
        motor_connected[index] = connected;

        let _ = server.publish_f64(motor_id(index, MOTOR_FIELD_OUTPUT), output);
        let _ = server.publish_f64(motor_id(index, MOTOR_FIELD_VELOCITY_RPS), velocity_rps);
        let _ = server.publish_f64(
            motor_id(index, MOTOR_FIELD_POSITION_ROT),
            state.motor_position[index],
        );
        let _ = server.publish_f64(motor_id(index, MOTOR_FIELD_CURRENT_A), current_a);
        let _ = server.publish_f64(
            motor_id(index, MOTOR_FIELD_TEMPERATURE_C),
            state.motor_temp_c[index],
        );
        let _ = server.publish_f64(motor_id(index, MOTOR_FIELD_VOLTAGE_V), voltage_v.max(0.0));
        let _ = server.publish_bool(motor_id(index, MOTOR_FIELD_CONNECTED), connected);
        let _ = server.publish_string(
            motor_id(index, MOTOR_FIELD_NEUTRAL_MODE),
            neutral_mode.to_string(),
        );
        let _ = server.publish_f64(motor_id(index, MOTOR_FIELD_COMMAND), command);
        let _ = server.publish_i64(motor_id(index, MOTOR_FIELD_CAN_ID), state.motor_can_id[index]);
        let _ = server.publish_string(
            motor_id(index, MOTOR_FIELD_CANBUS),
            state.motor_canbus[index].clone(),
        );
        let _ = server.publish_string(
            motor_id(index, MOTOR_FIELD_TYPE),
            MOTOR_TYPES[index].to_string(),
        );
        let _ = server.publish_bool(motor_id(index, MOTOR_FIELD_STALLED), stalled);
        let _ = server.publish_f64(motor_id(index, MOTOR_FIELD_CURRENT_LIMIT_A), current_limit);
        let _ = server.publish_bool(
            motor_id(index, MOTOR_FIELD_INVERTED),
            state.motor_inverted[index],
        );
        let _ = server.publish_bool(
            motor_id(index, MOTOR_FIELD_BRAKE_MODE),
            state.motor_brake_mode[index],
        );
        let _ = server.publish_f64(
            motor_output_command_id(index, MOTOR_OUTPUT_COMMAND_PERCENT),
            command,
        );
        let _ = server.publish_f64(
            motor_output_command_id(index, MOTOR_OUTPUT_COMMAND_VOLTAGE_V),
            command * MOTOR_COMMAND_MAX_VOLTAGE_V,
        );
        let _ = server.publish_f64(
            motor_output_command_id(index, MOTOR_OUTPUT_COMMAND_VELOCITY_RPS),
            command * MOTOR_COMMAND_MAX_VELOCITY_RPS,
        );
    }

    for index in 0..ENCODER_NAMES.len() {
        let ratio = state.encoder_ratio[index];
        let offset = state.encoder_offset[index];
        let direction = if state.encoder_inverted[index] {
            -1.0
        } else {
            1.0
        };
        let position = direction * (state.motor_position[index] * ratio + offset);
        let velocity = direction * motor_velocity[index] * ratio;
        let absolute = position.rem_euclid(1.0);
        let raw_absolute = (absolute + (phase * 1.8 + index as f64).sin() * 0.0018).rem_euclid(1.0);

        let _ = server.publish_f64(encoder_id(index, ENCODER_FIELD_POSITION_ROT), position);
        let _ = server.publish_f64(encoder_id(index, ENCODER_FIELD_VELOCITY_RPS), velocity);
        let _ = server.publish_f64(encoder_id(index, ENCODER_FIELD_ABSOLUTE_ROT), absolute);
        let _ = server.publish_bool(
            encoder_id(index, ENCODER_FIELD_CONNECTED),
            motor_connected[index],
        );
        let _ = server.publish_f64(encoder_id(index, ENCODER_FIELD_GEAR_RATIO), ratio);
        let _ = server.publish_f64(encoder_id(index, ENCODER_FIELD_OFFSET_ROT), offset);
        let _ = server.publish_i64(
            encoder_id(index, ENCODER_FIELD_CAN_ID),
            state.encoder_can_id[index],
        );
        let _ = server.publish_string(
            encoder_id(index, ENCODER_FIELD_CANBUS),
            state.encoder_canbus[index].clone(),
        );
        let _ = server.publish_string(
            encoder_id(index, ENCODER_FIELD_TYPE),
            ENCODER_TYPES[index].to_string(),
        );
        let _ = server.publish_bool(
            encoder_id(index, ENCODER_FIELD_INVERTED),
            state.encoder_inverted[index],
        );
        let _ = server.publish_bool(encoder_id(index, ENCODER_FIELD_SUPPORTS_SIMULATION), true);
        let _ = server.publish_f64(
            encoder_id(index, ENCODER_FIELD_RAW_ABSOLUTE_ROT),
            raw_absolute,
        );
    }

    let heading_raw = (phase * 82.0).rem_euclid(360.0);
    let heading = if state.imu_inverted {
        (360.0 - heading_raw).rem_euclid(360.0)
    } else {
        heading_raw
    };
    let yaw = heading;
    let roll = (phase * 0.8).sin() * 18.0;
    let pitch = (phase * 0.65).cos() * 13.0;
    let accel_x = (phase * 1.5).sin() * 0.9;
    let accel_y = (phase * 1.3).cos() * 0.7;
    let accel_z = 9.81 + (phase * 0.9).sin() * 0.12;
    let mag_x = (phase * 0.35).cos() * 38.0;
    let mag_y = (phase * 0.42).sin() * 29.0;
    let mag_z = 44.0 + (phase * 0.27).sin() * 6.0;
    let invert_sign = if state.imu_inverted { -1.0 } else { 1.0 };
    let gyro_x = invert_sign * (phase * 0.8).cos() * 32.0;
    let gyro_y = invert_sign * (-(phase * 0.65).sin() * 24.0);
    let gyro_z = invert_sign * 82.0;
    let angular_accel_z = invert_sign * (phase * 0.45).cos() * 47.0;
    let x_speed_rad_per_sec = gyro_x.to_radians();
    let y_speed_rad_per_sec = gyro_y.to_radians();
    let theta_speed_rad_per_sec = gyro_z.to_radians();
    let movement_speed_rad_per_sec = x_speed_rad_per_sec.hypot(y_speed_rad_per_sec);
    let x_speed_mps = (phase * 1.12).sin() * 2.3;
    let y_speed_mps = (phase * 0.84).cos() * 1.7;
    let movement_speed_mps = x_speed_mps.hypot(y_speed_mps);
    let movement_speed_normalized = (movement_speed_mps / state.imu_max_linear_speed.max(0.001))
        .abs()
        .clamp(0.0, 1.0);
    let normalized_speed =
        (theta_speed_rad_per_sec.abs() / state.imu_max_radial_speed.max(0.001)).clamp(0.0, 1.0);

    let _ = server.publish_f64(SIG_IMU_ROLL_DEG, roll);
    let _ = server.publish_f64(SIG_IMU_PITCH_DEG, pitch);
    let _ = server.publish_f64(SIG_IMU_YAW_DEG, yaw);
    let _ = server.publish_f64(SIG_IMU_HEADING_DEG, heading);
    let _ = server.publish_f64_array(SIG_IMU_ACCEL_XYZ, vec![accel_x, accel_y, accel_z]);
    let _ = server.publish_f64_array(SIG_IMU_GYRO_XYZ_DPS, vec![gyro_x, gyro_y, gyro_z]);
    let _ = server.publish_f64_array(SIG_IMU_MAG_XYZ_UT, vec![mag_x, mag_y, mag_z]);
    let _ = server.publish_bool(SIG_IMU_CONNECTED, true);
    let _ = server.publish_bool(SIG_IMU_INVERTED, state.imu_inverted);
    let _ = server.publish_i64(SIG_IMU_CAN_ID, state.imu_can_id);
    let _ = server.publish_string(SIG_IMU_CANBUS, state.imu_canbus.clone());
    let _ = server.publish_string(SIG_IMU_TYPE, IMU_TYPE.to_string());
    let _ = server.publish_f64(SIG_IMU_MAX_LINEAR_SPEED, state.imu_max_linear_speed);
    let _ = server.publish_f64(SIG_IMU_MAX_RADIAL_SPEED, state.imu_max_radial_speed);
    let _ = server.publish_f64(SIG_IMU_MAX_SPEED_WINDOW_SEC, state.imu_max_speed_window_sec);
    let _ = server.publish_f64(SIG_IMU_ANGULAR_ACCEL_Z_DPS2, angular_accel_z);
    let _ = server.publish_f64(SIG_IMU_X_SPEED_RAD_PER_SEC, x_speed_rad_per_sec);
    let _ = server.publish_f64(SIG_IMU_Y_SPEED_RAD_PER_SEC, y_speed_rad_per_sec);
    let _ = server.publish_f64(SIG_IMU_THETA_SPEED_RAD_PER_SEC, theta_speed_rad_per_sec);
    let _ = server.publish_f64(
        SIG_IMU_MOVEMENT_SPEED_RAD_PER_SEC,
        movement_speed_rad_per_sec,
    );
    let _ = server.publish_f64(SIG_IMU_X_SPEED_MPS, x_speed_mps);
    let _ = server.publish_f64(SIG_IMU_Y_SPEED_MPS, y_speed_mps);
    let _ = server.publish_f64(SIG_IMU_MOVEMENT_SPEED_MPS, movement_speed_mps);
    let _ = server.publish_f64(SIG_IMU_MOVEMENT_SPEED_NORMALIZED, movement_speed_normalized);
    let _ = server.publish_f64(SIG_IMU_NORMALIZED_SPEED, normalized_speed);
    let _ = server.publish_f64(SIG_IMU_VEL_X_DPS, gyro_x);
    let _ = server.publish_f64(SIG_IMU_VEL_Y_DPS, gyro_y);
    let _ = server.publish_f64(SIG_IMU_VEL_Z_DPS, gyro_z);
}

fn publish_low_rate(server: &ArcpServer, tick: u64, phase: f64, state: &SimState) {
    let cycle = ((tick / 100) % 4) as usize;
    let active_motor = MOTOR_NAMES[cycle];
    let status_message = format!(
        "active={} cmd={:.2} heading={:.1}",
        active_motor,
        state.motor_command[cycle],
        (phase * 82.0).rem_euclid(360.0)
    );

    let _ = server.publish_string(SIG_HW_STATUS, status_message);
}

fn apply_tunable(state: &mut SimState, signal_id: u16, value: SignalValue) {
    for index in 0..MOTOR_NAMES.len() {
        if signal_id == motor_id(index, MOTOR_FIELD_COMMAND) {
            if let Some(next) = signal_value_as_f64(&value) {
                state.motor_command[index] = next.clamp(-1.0, 1.0);
            }
            return;
        }
        if signal_id == motor_output_command_id(index, MOTOR_OUTPUT_COMMAND_PERCENT) {
            if let Some(next) = signal_value_as_f64(&value) {
                state.motor_command[index] = next.clamp(-1.0, 1.0);
            }
            return;
        }
        if signal_id == motor_output_command_id(index, MOTOR_OUTPUT_COMMAND_VOLTAGE_V) {
            if let Some(next) = signal_value_as_f64(&value) {
                state.motor_command[index] =
                    (next / MOTOR_COMMAND_MAX_VOLTAGE_V).clamp(-1.0, 1.0);
            }
            return;
        }
        if signal_id == motor_output_command_id(index, MOTOR_OUTPUT_COMMAND_VELOCITY_RPS) {
            if let Some(next) = signal_value_as_f64(&value) {
                state.motor_command[index] =
                    (next / MOTOR_COMMAND_MAX_VELOCITY_RPS).clamp(-1.0, 1.0);
            }
            return;
        }
        if signal_id == motor_id(index, MOTOR_FIELD_CURRENT_LIMIT_A) {
            if let Some(next) = signal_value_as_f64(&value) {
                state.motor_current_limit_a[index] = next.clamp(1.0, 200.0);
            }
            return;
        }
        if signal_id == motor_id(index, MOTOR_FIELD_CAN_ID) {
            if let Some(next) = signal_value_as_f64(&value) {
                state.motor_can_id[index] = next.round().clamp(0.0, 2048.0) as i64;
            }
            return;
        }
        if signal_id == motor_id(index, MOTOR_FIELD_CANBUS) {
            if let Some(next) = signal_value_as_string(&value) {
                let trimmed = next.trim();
                if !trimmed.is_empty() {
                    state.motor_canbus[index] = trimmed.to_string();
                }
            }
            return;
        }
        if signal_id == motor_id(index, MOTOR_FIELD_INVERTED) {
            if let Some(next) = signal_value_as_bool(&value) {
                state.motor_inverted[index] = next;
            }
            return;
        }
        if signal_id == motor_id(index, MOTOR_FIELD_BRAKE_MODE) {
            if let Some(next) = signal_value_as_bool(&value) {
                state.motor_brake_mode[index] = next;
            }
            return;
        }
    }

    for index in 0..ENCODER_NAMES.len() {
        if signal_id == encoder_id(index, ENCODER_FIELD_GEAR_RATIO) {
            if let Some(next) = signal_value_as_f64(&value) {
                state.encoder_ratio[index] = next.clamp(0.01, 100.0);
            }
            return;
        }
        if signal_id == encoder_id(index, ENCODER_FIELD_OFFSET_ROT) {
            if let Some(next) = signal_value_as_f64(&value) {
                state.encoder_offset[index] = next.clamp(-10.0, 10.0);
            }
            return;
        }
        if signal_id == encoder_id(index, ENCODER_FIELD_CAN_ID) {
            if let Some(next) = signal_value_as_f64(&value) {
                state.encoder_can_id[index] = next.round().clamp(0.0, 2048.0) as i64;
            }
            return;
        }
        if signal_id == encoder_id(index, ENCODER_FIELD_CANBUS) {
            if let Some(next) = signal_value_as_string(&value) {
                let trimmed = next.trim();
                if !trimmed.is_empty() {
                    state.encoder_canbus[index] = trimmed.to_string();
                }
            }
            return;
        }
        if signal_id == encoder_id(index, ENCODER_FIELD_INVERTED) {
            if let Some(next) = signal_value_as_bool(&value) {
                state.encoder_inverted[index] = next;
            }
            return;
        }
    }

    if signal_id == SIG_IMU_INVERTED {
        if let Some(next) = signal_value_as_bool(&value) {
            state.imu_inverted = next;
        }
        return;
    }

    if signal_id == SIG_IMU_CAN_ID {
        if let Some(next) = signal_value_as_f64(&value) {
            state.imu_can_id = next.round().clamp(0.0, 2048.0) as i64;
        }
        return;
    }

    if signal_id == SIG_IMU_CANBUS {
        if let Some(next) = signal_value_as_string(&value) {
            let trimmed = next.trim();
            if !trimmed.is_empty() {
                state.imu_canbus = trimmed.to_string();
            }
        }
        return;
    }

    if signal_id == SIG_IMU_MAX_LINEAR_SPEED {
        if let Some(next) = signal_value_as_f64(&value) {
            state.imu_max_linear_speed = next.clamp(0.1, 50.0);
        }
        return;
    }

    if signal_id == SIG_IMU_MAX_RADIAL_SPEED {
        if let Some(next) = signal_value_as_f64(&value) {
            state.imu_max_radial_speed = next.clamp(0.1, 200.0);
        }
        return;
    }

    if signal_id == SIG_IMU_MAX_SPEED_WINDOW_SEC {
        if let Some(next) = signal_value_as_f64(&value) {
            state.imu_max_speed_window_sec = next.clamp(0.05, 5.0);
        }
    }
}

fn signal_value_as_f64(value: &SignalValue) -> Option<f64> {
    match value {
        SignalValue::F64(next) => Some(*next),
        SignalValue::I64(next) => Some(*next as f64),
        SignalValue::Str(next) => next.trim().parse::<f64>().ok(),
        _ => None,
    }
}

fn signal_value_as_string(value: &SignalValue) -> Option<String> {
    match value {
        SignalValue::Str(next) => Some(next.clone()),
        SignalValue::I64(next) => Some(next.to_string()),
        SignalValue::F64(next) => Some(next.to_string()),
        SignalValue::Bool(next) => Some(next.to_string()),
        _ => None,
    }
}

fn signal_value_as_bool(value: &SignalValue) -> Option<bool> {
    match value {
        SignalValue::Bool(next) => Some(*next),
        SignalValue::I64(next) => Some(*next != 0),
        SignalValue::F64(next) => Some(*next != 0.0),
        SignalValue::Str(next) => match next.trim().to_ascii_lowercase().as_str() {
            "true" | "1" | "yes" | "on" | "brake" => Some(true),
            "false" | "0" | "no" | "off" | "coast" => Some(false),
            _ => None,
        },
        _ => None,
    }
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
