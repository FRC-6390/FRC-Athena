package ca.frc6390.athena.drivetrains.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

import ca.frc6390.athena.drivetrains.DrivetrainSpeedSectionBase;
import ca.frc6390.athena.drivetrains.DrivetrainSpeedConfigSupport;
import ca.frc6390.athena.drivetrains.SectionedDrivetrainConfig;
import ca.frc6390.athena.core.sections.SectionedAccess;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DriveIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.EncoderIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.SteerIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.hardware.imu.AthenaImu;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.VirtualImu;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.sim.SwerveSimulationConfig;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Builder-style configuration object for constructing a {@link SwerveDrivetrain}. It captures module
 * hardware layouts, controller gains, CAN IDs, and optional simulation parameters so the drivetrain
 * can be created consistently across both robot code and tests.
 */
public class SwerveDrivetrainConfig extends SectionedDrivetrainConfig<SwerveDrivetrainConfig>
        implements RobotDrivetrainConfig<SwerveDrivetrain> {
    private static final String CONNECTIVITY_FAIL_FAST_PROPERTY =
            "athena.startup.connectivity.failFast";
    private static final String CONNECTIVITY_ASYNC_PROPERTY =
            "athena.startup.connectivity.async";
    private static final String CONNECTIVITY_TIMEOUT_MS_PROPERTY =
            "athena.startup.connectivity.timeoutMs";
    private static final String CONNECTIVITY_PARALLELISM_PROPERTY =
            "athena.startup.connectivity.parallelism";
    private static final long DEFAULT_CONNECTIVITY_TIMEOUT_MS = 250L;
    private static final int MAX_CONNECTIVITY_CHECK_THREADS = 8;
    private static final AtomicInteger CONNECTIVITY_THREAD_COUNTER = new AtomicInteger(1);
    
    /** Optional IMU configuration that provides field-centric heading. */
    private ImuConfig imu = null;
    /** Per-module hardware configuration (drive/steer motors, encoders). */
    private SwerveModuleConfig[] modules = null;
    /** PID controller used to counter uncommanded drift while driving straight. */
    private PIDController driftPID = null;
    /** Speed threshold (m/s) where drift compensation engages. */
    private double driftActivationSpeed = 0.05;
    /** Whether open-loop driving defaults to field-relative chassis speeds. */
    private boolean fieldRelative = true;
    /** CAN IDs for drive motors ordered FL, FR, BL, BR (negatives invert motors). */
    private int[] driveIds = DriveIDs.SWERVE_CHASSIS_STANDARD.getIDs();
    /** CAN IDs for steer motors ordered FL, FR, BL, BR (negatives invert motors). */
    private int[] steerIDs = SteerIDs.SWERVE_CHASSIS_STANDARD.getIDs();
    /** CAN IDs for steering encoders ordered FL, FR, BL, BR (negatives invert encoders). */
    private int[] encoderIds = EncoderIDs.SWERVE_CHASSIS_STANDARD.getIDs();
    /** Device ID of the inertial gyro used for heading. */
    private int gryoId = DrivetrainIDs.SWERVE_CHASSIS_STANDARD.getGyro();
    /** Legacy/default steer inversion used when neither module nor drivetrain overrides are explicit. */
    private boolean steerInverted = true;
    /** True once the drivetrain-level steer inversion is explicitly configured. */
    private boolean steerInvertedConfigured = false;
    /** Legacy/default encoder inversion used when neither module nor drivetrain overrides are explicit. */
    private boolean encoderInverted = false;
    /** True once the drivetrain-level encoder inversion is explicitly configured. */
    private boolean encoderInvertedConfigured = false;
    /** Legacy/default drive inversion used when neither module nor drivetrain overrides are explicit. */
    private boolean driveInverted = false;
    /** True once the drivetrain-level drive inversion is explicitly configured. */
    private boolean driveInvertedConfigured = false;
    /** PID controller that maintains heading when commanded setpoints demand rotation. */
    private PIDController rotationPID = null;
    /** Optional drive feedforward used to command wheel velocities as voltages. */
    private SimpleMotorFeedforward driveFeedforward = null;
    /** Whether the configured drive feedforward is enabled. */
    private boolean driveFeedforwardEnabled = true;
    /** Whether modules apply cosine speed scaling while steering to target angles. */
    private boolean cosineCompensationEnabled = true;
    /** Minimum steering error (degrees) required before cosine compensation applies. */
    private double cosineCompensationErrorDegrees = 0.0;
    /** Drive-side current limit applied per motor (amps). */
    private double driveCurrentLimit = 80;
    /** Steer-side current limit applied per motor (amps). */
    private double steerCurrentLimit = 40;
    /** Absolute encoder offsets (radians) for each module. */
    private double[] encoderOffsets = {0,0,0,0};
    /** CAN bus identifier shared by all configured devices. */
    private String canbus = "rio";
    /** Optional CAN bus override used specifically by the IMU. */
    private String imuCanbus = null;
    /** Module corner locations relative to the robot center. */
    private Translation2d[] locations = null;
    /** Optional physics configuration used when running drivetrain simulation. */
    private SwerveSimulationConfig simulationConfig = SwerveSimulationConfig.defaults();
    /** Shared speed source/blend registration store used by speed() sections. */
    private final DrivetrainSpeedConfigSupport speedConfig = new DrivetrainSpeedConfigSupport();

    public static final class CosineCompensationSection {
        private boolean enabled;
        private double errorDegrees;

        private CosineCompensationSection(boolean enabled, double errorDegrees) {
            this.enabled = enabled;
            this.errorDegrees = errorDegrees;
        }

        public CosineCompensationSection enabled(boolean enabled) {
            this.enabled = enabled;
            return this;
        }

        public CosineCompensationSection error(double errorDegrees) {
            this.errorDegrees = errorDegrees;
            return this;
        }

        private boolean enabled() {
            return enabled;
        }

        private double error() {
            return errorDegrees;
        }
    }

    public static SwerveDrivetrainConfig create() {
        return new SwerveDrivetrainConfig();
    }

    @Override
    protected SwerveDrivetrainConfig self() {
        return this;
    }

    public SwerveDrivetrainConfig hardware(Consumer<HardwareSection> section) {
        return applySection(section, HardwareSection::new);
    }

    public HardwareSection hardware() {
        return new HardwareSection();
    }

    public SwerveDrivetrainConfig control(Consumer<ControlSection> section) {
        return applySection(section, ControlSection::new);
    }

    public ControlSection control() {
        return new ControlSection();
    }

    public SwerveDrivetrainConfig simulation(Consumer<SimulationSection> section) {
        return applySection(section, SimulationSection::new);
    }

    public SimulationSection simulation() {
        return new SimulationSection();
    }

    public SwerveDrivetrainConfig speed(Consumer<SpeedSection> section) {
        return applySection(section, SpeedSection::new);
    }

    public SpeedSection speed() {
        return new SpeedSection();
    }

    public SwerveDrivetrainConfig config(Consumer<ConfigSection> section) {
        return applySection(section, ConfigSection::new);
    }

    public ConfigSection config() {
        return new ConfigSection();
    }

    public final class HardwareSection {
        public HardwareSection imu(AthenaImu imuType, boolean inverted) {
            applyImu(imuType, inverted);
            return this;
        }

        public HardwareSection modules(SwerveModuleConfig... moduleConfigs) {
            SwerveDrivetrainConfig.this.modules(moduleConfigs);
            return this;
        }

        public HardwareSection driveIds(int[] ids) {
            applyDriveIds(ids);
            return this;
        }

        public HardwareSection steerIds(int[] ids) {
            applySteerIds(ids);
            return this;
        }

        public HardwareSection encoderIds(int[] ids) {
            applyEncoderIds(ids);
            return this;
        }

        public HardwareSection driveIds(DriveIDs ids) {
            applyDriveIds(ids);
            return this;
        }

        public HardwareSection steerIds(SteerIDs ids) {
            applySteerIds(ids);
            return this;
        }

        public HardwareSection encoderIds(EncoderIDs ids) {
            applyEncoderIds(ids);
            return this;
        }

        public HardwareSection imuId(int id) {
            applyImuId(id);
            return this;
        }

        public HardwareSection ids(DrivetrainIDs ids) {
            applyIds(ids);
            return this;
        }

        public HardwareSection driveInverted(boolean inverted) {
            applyDriveInverted(inverted);
            return this;
        }

        public HardwareSection steerInverted(boolean inverted) {
            applySteerInverted(inverted);
            return this;
        }

        public HardwareSection encoderInverted(boolean inverted) {
            applyEncoderInverted(inverted);
            return this;
        }

        public HardwareSection currentLimit(double amps) {
            applyCurrentLimit(amps);
            return this;
        }

        public HardwareSection currentLimit(double driveAmps, double steerAmps) {
            applyCurrentLimit(driveAmps, steerAmps);
            return this;
        }

        public HardwareSection driveCurrentLimit(double amps) {
            applyDriveCurrentLimit(amps);
            return this;
        }

        public HardwareSection steerCurrentLimit(double amps) {
            applySteerCurrentLimit(amps);
            return this;
        }

        public HardwareSection encoderOffsets(double... offsets) {
            applyEncoderOffsets(offsets);
            return this;
        }

        public HardwareSection canbus(String bus) {
            applyCanbus(bus);
            return this;
        }

        public HardwareSection imuCanbus(String bus) {
            applyImuCanbus(bus);
            return this;
        }

        public HardwareSection moduleLocations(Translation2d[] moduleLocations) {
            applyModuleLocations(moduleLocations);
            return this;
        }

        public HardwareSection moduleLocations(double trackWidthMeters, double wheelbaseMeters) {
            applyModuleLocations(trackWidthMeters, wheelbaseMeters);
            return this;
        }

        public HardwareSection moduleLocations(double trackWidthMeters) {
            applyModuleLocations(trackWidthMeters);
            return this;
        }
    }

    public final class ControlSection {
        public ControlSection driftPid(double kP, double kI, double kD) {
            applyDriftPid(kP, kI, kD);
            return this;
        }

        public ControlSection driftPid(PIDController pid) {
            applyDriftPid(pid);
            return this;
        }

        public ControlSection driftActivationSpeed(double speedMetersPerSecond) {
            applyDriftActivationSpeed(speedMetersPerSecond);
            return this;
        }

        public ControlSection fieldRelative(boolean enabled) {
            applyFieldRelative(enabled);
            return this;
        }

        public ControlSection rotationPid(double kP, double kI, double kD) {
            applyRotationPid(kP, kI, kD);
            return this;
        }

        public ControlSection rotationPid(PIDController pid) {
            applyRotationPid(pid);
            return this;
        }

        public ControlSection driveFeedforward(SimpleMotorFeedforward feedforward) {
            applyDriveFeedforward(feedforward);
            return this;
        }

        public ControlSection driveFeedforward(double kS, double kV, double kA) {
            applyDriveFeedforward(kS, kV, kA);
            return this;
        }

        public ControlSection driveFeedforwardEnabled(boolean enabled) {
            applyDriveFeedforwardEnabled(enabled);
            return this;
        }

        public ControlSection cosineCompensation(boolean enabled) {
            applyCosineCompensation(enabled);
            return this;
        }

        public ControlSection cosineCompensation(Consumer<CosineCompensationSection> section) {
            CosineCompensationSection resolved =
                    new CosineCompensationSection(cosineCompensationEnabled, cosineCompensationErrorDegrees);
            if (section != null) {
                section.accept(resolved);
            }
            applyCosineCompensation(resolved.enabled());
            applyCosineCompensationError(resolved.error());
            return this;
        }
    }

    public final class SimulationSection {
        public SimulationSection config(SwerveSimulationConfig simulation) {
            applySimulationConfig(simulation);
            return this;
        }
    }

    public final class ConfigSection {
        public ConfigSection imu(AthenaImu imuType, boolean inverted) {
            applyImu(imuType, inverted);
            return this;
        }

        public ConfigSection modules(SwerveModuleConfig... moduleConfigs) {
            SwerveDrivetrainConfig.this.modules(moduleConfigs);
            return this;
        }

        public ConfigSection driftPid(double kP, double kI, double kD) {
            applyDriftPid(kP, kI, kD);
            return this;
        }

        public ConfigSection driftPid(PIDController pid) {
            applyDriftPid(pid);
            return this;
        }

        public ConfigSection driftActivationSpeed(double speedMetersPerSecond) {
            applyDriftActivationSpeed(speedMetersPerSecond);
            return this;
        }

        public ConfigSection fieldRelative(boolean enabled) {
            applyFieldRelative(enabled);
            return this;
        }

        public ConfigSection driveIds(int[] ids) {
            applyDriveIds(ids);
            return this;
        }

        public ConfigSection steerIds(int[] ids) {
            applySteerIds(ids);
            return this;
        }

        public ConfigSection encoderIds(int[] ids) {
            applyEncoderIds(ids);
            return this;
        }

        public ConfigSection driveIds(DriveIDs ids) {
            applyDriveIds(ids);
            return this;
        }

        public ConfigSection steerIds(SteerIDs ids) {
            applySteerIds(ids);
            return this;
        }

        public ConfigSection encoderIds(EncoderIDs ids) {
            applyEncoderIds(ids);
            return this;
        }

        public ConfigSection imuId(int id) {
            applyImuId(id);
            return this;
        }

        public ConfigSection ids(DrivetrainIDs ids) {
            applyIds(ids);
            return this;
        }

        public ConfigSection driveInverted(boolean inverted) {
            applyDriveInverted(inverted);
            return this;
        }

        public ConfigSection steerInverted(boolean inverted) {
            applySteerInverted(inverted);
            return this;
        }

        public ConfigSection encoderInverted(boolean inverted) {
            applyEncoderInverted(inverted);
            return this;
        }

        public ConfigSection rotationPid(double kP, double kI, double kD) {
            applyRotationPid(kP, kI, kD);
            return this;
        }

        public ConfigSection rotationPid(PIDController pid) {
            applyRotationPid(pid);
            return this;
        }

        public ConfigSection driveFeedforward(SimpleMotorFeedforward feedforward) {
            applyDriveFeedforward(feedforward);
            return this;
        }

        public ConfigSection driveFeedforward(double kS, double kV, double kA) {
            applyDriveFeedforward(kS, kV, kA);
            return this;
        }

        public ConfigSection driveFeedforwardEnabled(boolean enabled) {
            applyDriveFeedforwardEnabled(enabled);
            return this;
        }

        public ConfigSection cosineCompensation(boolean enabled) {
            applyCosineCompensation(enabled);
            return this;
        }

        public ConfigSection cosineCompensation(Consumer<CosineCompensationSection> section) {
            CosineCompensationSection resolved =
                    new CosineCompensationSection(cosineCompensationEnabled, cosineCompensationErrorDegrees);
            if (section != null) {
                section.accept(resolved);
            }
            applyCosineCompensation(resolved.enabled());
            applyCosineCompensationError(resolved.error());
            return this;
        }

        public ConfigSection currentLimit(double amps) {
            applyCurrentLimit(amps);
            return this;
        }

        public ConfigSection currentLimit(double driveAmps, double steerAmps) {
            applyCurrentLimit(driveAmps, steerAmps);
            return this;
        }

        public ConfigSection driveCurrentLimit(double amps) {
            applyDriveCurrentLimit(amps);
            return this;
        }

        public ConfigSection steerCurrentLimit(double amps) {
            applySteerCurrentLimit(amps);
            return this;
        }

        public ConfigSection encoderOffsets(double... offsets) {
            applyEncoderOffsets(offsets);
            return this;
        }

        public ConfigSection canbus(String bus) {
            applyCanbus(bus);
            return this;
        }

        public ConfigSection imuCanbus(String bus) {
            applyImuCanbus(bus);
            return this;
        }

        public ConfigSection moduleLocations(Translation2d[] moduleLocations) {
            applyModuleLocations(moduleLocations);
            return this;
        }

        public ConfigSection moduleLocations(double trackWidthMeters, double wheelbaseMeters) {
            applyModuleLocations(trackWidthMeters, wheelbaseMeters);
            return this;
        }

        public ConfigSection moduleLocations(double trackWidthMeters) {
            applyModuleLocations(trackWidthMeters);
            return this;
        }

        public ConfigSection simulationConfig(SwerveSimulationConfig simulation) {
            applySimulationConfig(simulation);
            return this;
        }

        public ConfigSection speed(Consumer<SpeedSection> section) {
            return SectionedAccess.apply(this, section, SpeedSection::new);
        }

        public SpeedSection speed() {
            return new SpeedSection();
        }
    }

    public final class SpeedSection extends DrivetrainSpeedSectionBase<SpeedSection> {
        private SpeedSection() {
            super(speedConfig);
        }

        @Override
        protected SpeedSection self() {
            return this;
        }
    }

    /**
     * Creates a configuration with default hardware IDs and explicit module locations.
     *
     * @param locations module corner offsets relative to robot center (FL, FR, BL, BR)
     */
    public static SwerveDrivetrainConfig defaults(Translation2d[] locations){
        return create().hardware(h -> h.moduleLocations(locations));
    }

    /**
     * Creates a configuration with default hardware IDs and square module footprint.
     *
     * @param trackWidth distance (meters) between left and right modules
     */
    public static SwerveDrivetrainConfig defaults(double trackWidth){
        return create().hardware(h -> h.moduleLocations(trackWidth));
    }

    /**
     * Creates a configuration with default hardware IDs and rectangular footprint.
     *
     * @param trackWidth distance (meters) between left and right modules
     * @param wheelbase distance (meters) between front and back modules
     */
    public static SwerveDrivetrainConfig defaults(double trackWidth, double wheelbase){
        return create().hardware(h -> h.moduleLocations(trackWidth, wheelbase));
    }

    /**
     * Configures the standard team drivetrain IDs while using the provided module locations.
     */
    public static SwerveDrivetrainConfig standard(Translation2d[] locations){
        return SwerveDrivetrainConfig.defaults(locations)
                .hardware(h -> h.ids(DrivetrainIDs.SWERVE_CHASSIS_STANDARD));
    }

    /**
     * Configures the standard team drivetrain IDs while assuming a square footprint.
     */
    public static SwerveDrivetrainConfig standard(double trackWidth){
        return SwerveDrivetrainConfig.defaults(trackWidth)
                .hardware(h -> h.ids(DrivetrainIDs.SWERVE_CHASSIS_STANDARD));
    }

    /**
     * Configures the standard team drivetrain IDs while assuming a rectangular footprint.
     */
    public static SwerveDrivetrainConfig standard(double trackWidth, double wheelbase){
        return SwerveDrivetrainConfig.defaults(trackWidth, wheelbase)
                .hardware(h -> h.ids(DrivetrainIDs.SWERVE_CHASSIS_STANDARD));
    }

    /**
     * Declares the gyro hardware to use along with its inversion.
     *
     * @param imu IMU type installed on the robot
     * @param inverted true if yaw should be inverted
     */
    private SwerveDrivetrainConfig applyImu(AthenaImu imu, boolean inverted){
        this.imu = ImuConfig
                .create(imu.resolve(), DrivetrainIDs.SWERVE_CHASSIS_STANDARD.getGyro())
                .hardware(h -> h.inverted(inverted));
        return this;
    }

    /**
     * Registers the swerve module configurations. Passing a single module copies it to all four
     * module positions in order FL, FR, BL, BR.
     *
     * @param modules one module definition per corner, or a single definition to clone
     */
    private SwerveDrivetrainConfig modules(SwerveModuleConfig... modules){
        if (modules.length == 1) {
            modules = new SwerveModuleConfig[]{
                new SwerveModuleConfig(modules[0]),
                new SwerveModuleConfig(modules[0]),
                new SwerveModuleConfig(modules[0]),
                new SwerveModuleConfig(modules[0])
            };
        }
        this.modules = modules;
        return this;
    }

    /**
     * Creates a drift-correction PID controller using the provided gains.
     */
    private SwerveDrivetrainConfig applyDriftPid(double kP, double kI, double kd){
        return applyDriftPid(new PIDController(kP, kI, kd));
    }
    
    /**
     * Supplies a fully constructed drift-correction PID controller.
     */
    private SwerveDrivetrainConfig applyDriftPid(PIDController driftPID){
        this.driftPID = driftPID;
        return this;
    }

    /**
     * Sets the minimum drive speed (m/s) required before drift correction engages.
     */
    private SwerveDrivetrainConfig applyDriftActivationSpeed(double driftActivationSpeed){
        this.driftActivationSpeed = driftActivationSpeed;
        return this;
    }

    /**
     * Controls whether the drivetrain interprets inputs as field-relative by default.
     */
    private SwerveDrivetrainConfig applyFieldRelative(boolean fieldRelative){
        this.fieldRelative = fieldRelative;
        return this;
    }

    /**
     * Sets the CAN IDs used for drive motors (FL, FR, BL, BR). Negative IDs invert that motor.
     */
    private SwerveDrivetrainConfig applyDriveIds(int[] driveIds){
        this.driveIds = driveIds;
        return this;
    } 

    /**
     * Sets the CAN IDs used for steering motors (FL, FR, BL, BR). Negative IDs invert that motor.
     */
    private SwerveDrivetrainConfig applySteerIds(int[] steerIDs){
        this.steerIDs = steerIDs;
        return this;
    } 

    /**
     * Sets the CAN IDs used for steering encoders (FL, FR, BL, BR). Negative IDs invert that encoder.
     */
    private SwerveDrivetrainConfig applyEncoderIds(int[] encoderIds){
        this.encoderIds = encoderIds;
        return this;
    } 

    /**
     * Populates drive motor IDs from a {@link DriveIDs} preset.
     */
    private SwerveDrivetrainConfig applyDriveIds(DriveIDs driveIds){
       return applyDriveIds(driveIds.getIDs());
    } 

    /**
     * Populates steer motor IDs from a {@link SteerIDs} preset.
     */
    private SwerveDrivetrainConfig applySteerIds(SteerIDs steerIDs){
        return applySteerIds(steerIDs.getIDs());
    } 

    /**
     * Populates encoder IDs from an {@link EncoderIDs} preset.
     */
    private SwerveDrivetrainConfig applyEncoderIds(EncoderIDs encoderIds){
        return applyEncoderIds(encoderIds.getIDs());
    } 

    /**
     * Declares the CAN device ID used by the IMU.
     */
    private SwerveDrivetrainConfig applyImuId(int id){
        gryoId = id;
        return this;
    }

    /**
     * Applies a common {@link DrivetrainIDs} preset covering drive, steer, encoder, and gyro IDs.
     */
    private SwerveDrivetrainConfig applyIds(DrivetrainIDs ids){
        applyDriveIds(ids.getDrive());
        applySteerIds(ids.getSteer());
        applyEncoderIds(ids.getEncoders());
        applyImuId(ids.getGyro());
        return this;
    } 

    /**
     * Sets whether drive motors should be inverted.
     */
    private SwerveDrivetrainConfig applyDriveInverted(boolean driveInverted){
        this.driveInverted = driveInverted;
        this.driveInvertedConfigured = true;
        return this;
    } 

    /**
     * Sets whether steer motors should be inverted.
     */
    private SwerveDrivetrainConfig applySteerInverted(boolean steerInverted){
        this.steerInverted = steerInverted;
        this.steerInvertedConfigured = true;
        return this;
    } 

    /**
     * Sets whether steering encoders should be inverted.
     */
    private SwerveDrivetrainConfig applyEncoderInverted(boolean encoderInverted){
        this.encoderInverted = encoderInverted;
        this.encoderInvertedConfigured = true;
        return this;
    } 

    /**
     * Creates a rotation PID controller used for heading control with the provided gains.
     */
    private SwerveDrivetrainConfig applyRotationPid(double kP, double kI, double kd){
        return applyRotationPid(new PIDController(kP, kI, kd));
    }
    
    /**
     * Supplies a fully constructed rotation PID controller.
     */
    private SwerveDrivetrainConfig applyRotationPid(PIDController rotationPID){
        this.rotationPID = rotationPID;
        return this;
    }

    /**
     * Provides drive feedforward gains used to compute voltage setpoints.
     */
    private SwerveDrivetrainConfig applyDriveFeedforward(SimpleMotorFeedforward feedforward) {
        this.driveFeedforward = feedforward;
        return this;
    }

    /**
     * Convenience helper to configure drive feedforward gains.
     */
    private SwerveDrivetrainConfig applyDriveFeedforward(double kS, double kV, double kA) {
        return applyDriveFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    /**
     * Enables or disables the configured drive feedforward.
     */
    private SwerveDrivetrainConfig applyDriveFeedforwardEnabled(boolean enabled) {
        this.driveFeedforwardEnabled = enabled;
        return this;
    }

    /**
     * Enables or disables module cosine compensation that scales speed during steering transients.
     */
    private SwerveDrivetrainConfig applyCosineCompensation(boolean enabled) {
        this.cosineCompensationEnabled = enabled;
        return this;
    }

    /**
     * Sets the minimum steering error (degrees) before cosine compensation scales wheel speed.
     */
    private SwerveDrivetrainConfig applyCosineCompensationError(double errorDegrees) {
        if (!Double.isFinite(errorDegrees)) {
            return this;
        }
        this.cosineCompensationErrorDegrees = Math.max(0.0, errorDegrees);
        return this;
    }
    
    /**
     * Applies the same current limit to both drive and steer motors.
     */
    private SwerveDrivetrainConfig applyCurrentLimit(double currentLimit){
        return applyCurrentLimit(currentLimit, currentLimit);
    } 

    /**
     * Applies the same current limit to both drive and steer motors.
     */
    private SwerveDrivetrainConfig applyCurrentLimit(double driveCurrentLimit, double steerCurrentLimit){
        applyDriveCurrentLimit(driveCurrentLimit);
        applySteerCurrentLimit(steerCurrentLimit);
        return this;
    } 

    /**
     * Sets the drive motor current limit (amps).
     */
    private SwerveDrivetrainConfig applyDriveCurrentLimit(double currentLimit){
        this.driveCurrentLimit = currentLimit;
        return this;
    } 

    /**
     * Sets the steer motor current limit (amps).
     */
    private SwerveDrivetrainConfig applySteerCurrentLimit(double currentLimit){
        this.steerCurrentLimit = currentLimit;
        return this;
    } 


    /**
     * Provides absolute encoder offsets in radians (FL, FR, BL, BR).
     */
    private SwerveDrivetrainConfig applyEncoderOffsets(double... encoderOffsets){
        this.encoderOffsets = encoderOffsets;
        return this;
    } 

    /**
     * Sets the common CAN bus used by all motors, encoders, and sensors.
     */
    private SwerveDrivetrainConfig applyCanbus(String canbus){
        this.canbus = canbus;
        return this;
    } 

    /**
     * Sets an optional CAN bus override for the IMU only.
     */
    private SwerveDrivetrainConfig applyImuCanbus(String imuCanbus) {
        this.imuCanbus = imuCanbus;
        return this;
    }

    /**
     * Declares the physical module locations relative to robot center (FL, FR, BL, BR).
     */
    private SwerveDrivetrainConfig applyModuleLocations(Translation2d[] locations){
        this.locations = locations;
        return this;
    }

    /**
     * Declares module locations assuming a rectangular footprint (track width x wheelbase).
     */
    private SwerveDrivetrainConfig applyModuleLocations(double trackWidth, double wheelbase){
        return applyModuleLocations(SwerveModuleConfig.generateModuleLocations(trackWidth, wheelbase));
    }

    /**
     * Declares module locations assuming a square footprint (track width on both axes).
     */
    private SwerveDrivetrainConfig applyModuleLocations(double trackWidth){
        return applyModuleLocations(SwerveModuleConfig.generateModuleLocations(trackWidth, trackWidth));
    }

    /**
     * Overrides the drivetrain simulation configuration used when running in WPILib simulation.
     */
    private SwerveDrivetrainConfig applySimulationConfig(SwerveSimulationConfig simulationConfig) {
        this.simulationConfig = simulationConfig;
        return this;
    }

    /**
     * Applies all registered configuration options to the module definitions and constructs the
     * {@link SwerveDrivetrain}. Also wires up optional drift/rotation controllers and simulation
     * configuration.
     *
     * @throws Error if any module configuration arrays (IDs, offsets, locations) are mismatched in size
     */
    @Override
    public SwerveDrivetrain build() {
        if (encoderIds.length != modules.length) {
        throw new Error("ENCODER ID ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+encoderIds.length+")");
        }

        if (driveIds.length != modules.length) {
        throw new Error("DRIVE ID ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+driveIds.length+")");
        }

        if (steerIDs.length != modules.length) {
        throw new Error("STEER ID ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+steerIDs.length+")");
        }

        if (encoderOffsets.length != modules.length) {
        throw new Error("ENCODER OFFSETS ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+encoderOffsets.length+")");
        }

        if (locations.length != modules.length) {
        throw new Error("MODULE LOCATIONS ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+locations.length+")");
        }

        for (int i = 0; i < modules.length; i++) {
            int encoderId = resolvedModuleId(encoderIds[i]);
            int steerId = resolvedModuleId(steerIDs[i]);
            int driveId = resolvedModuleId(driveIds[i]);
            boolean resolvedEncoderInverted = resolvedModuleInverted(
                    encoderInverted,
                    modules[i].encoderInvertedExplicit(),
                    moduleEncoderInverted(modules[i]),
                    encoderInvertedConfigured,
                    encoderInverted,
                    encoderIds[i]);
            boolean resolvedSteerInverted = resolvedModuleInverted(
                    steerInverted,
                    modules[i].steerInvertedExplicit(),
                    moduleSteerInverted(modules[i]),
                    steerInvertedConfigured,
                    steerInverted,
                    steerIDs[i]);
            boolean resolvedDriveInverted = resolvedModuleInverted(
                    driveInverted,
                    modules[i].driveInvertedExplicit(),
                    moduleDriveInverted(modules[i]),
                    driveInvertedConfigured,
                    driveInverted,
                    driveIds[i]);
            modules[i] = modules[i].encoderId(encoderId)
                                    .steerId(steerId)
                                    .driveId(driveId)
                                    .encoderInverted(resolvedEncoderInverted)
                                    .steerInverted(resolvedSteerInverted)
                                    .driveInverted(resolvedDriveInverted)
                                    .steerCurrentLimit(steerCurrentLimit)
                                    .driveCurrentLimit(driveCurrentLimit)
                                    .offset(encoderOffsets[i])
                                    .canbus(canbus)
                                    .location(locations[i]);

            if(rotationPID != null){
                modules[i] = modules[i].pid(p -> p
                        .p(rotationPID.getP())
                        .i(rotationPID.getI())
                        .d(rotationPID.getD()));
            }
        } 
        String resolvedImuCanbus = imuCanbus != null ? imuCanbus : canbus;
        if (imu != null) {
            imu = imu.hardware(h -> h.id(gryoId).canbus(resolvedImuCanbus));
        }

        Imu resolvedImu = imu == null ? null : new VirtualImu(HardwareFactories.imu(imu));
        SwerveDrivetrain dt = new SwerveDrivetrain(resolvedImu, modules);
        ensureRequiredHardwareConnected(dt, resolvedImu);

        if (driveFeedforward != null) {
            dt.driveFeedforward(driveFeedforward);
            dt.driveFeedforwardEnabled(driveFeedforwardEnabled);
        }

        dt.cosineCompensationEnabled(cosineCompensationEnabled);
        dt.cosineCompensationError(cosineCompensationErrorDegrees);

        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            dt.configureSimulation(simulationConfig != null ? simulationConfig : SwerveSimulationConfig.defaults());
        }

        if (driftPID != null){
            dt.driftCorrectionPid(driftPID);
            dt.driftCorrectionEnabled(true);
        }

        dt.fieldRelative(fieldRelative);
        dt.driftActivationSpeed = driftActivationSpeed;
        speedConfig.apply(dt.robotSpeeds());
        return dt;
    }

    static int resolvedModuleId(int configuredId) {
        return Math.abs(configuredId);
    }

    static boolean resolvedModuleInverted(
            boolean defaultInverted,
            boolean moduleOverrideConfigured,
            boolean moduleInverted,
            boolean drivetrainOverrideConfigured,
            boolean drivetrainInverted,
            int configuredId) {
        boolean baseInverted = defaultInverted;
        if (moduleOverrideConfigured) {
            baseInverted = moduleInverted;
        }
        if (drivetrainOverrideConfigured) {
            baseInverted = drivetrainInverted;
        }
        return resolvedModuleInverted(baseInverted, configuredId);
    }

    static boolean resolvedModuleInverted(boolean baseInverted, int configuredId) {
        return baseInverted ^ (configuredId < 0);
    }

    private static boolean moduleDriveInverted(SwerveModuleConfig config) {
        return config != null && config.driveMotor() != null && config.driveMotor().inverted();
    }

    private static boolean moduleSteerInverted(SwerveModuleConfig config) {
        return config != null && config.rotationMotor() != null && config.rotationMotor().inverted();
    }

    private static boolean moduleEncoderInverted(SwerveModuleConfig config) {
        return config != null && config.encoder() != null && config.encoder().inverted();
    }

    private static void ensureRequiredHardwareConnected(SwerveDrivetrain drivetrain, Imu imuDevice) {
        if (RobotBase.isSimulation()) {
            return;
        }

        List<String> failures = new ArrayList<>();
        List<ConnectivityProbe> probes = new ArrayList<>();
        if (imuDevice == null) {
            failures.add("imu missing");
        } else {
            probes.add(new ConnectivityProbe("imu", () -> imuDevice.isConnected(true), describeImu(imuDevice)));
        }

        SwerveModule[] modules = drivetrain != null ? drivetrain.swerveModules : null;
        if (modules == null || modules.length == 0) {
            failures.add("no swerve modules configured");
        } else {
            for (int i = 0; i < modules.length; i++) {
                SwerveModule module = modules[i];
                if (module == null) {
                    failures.add("module[" + i + "] missing");
                    continue;
                }

                MotorController driveMotor = module.getDriveMotorController();
                if (driveMotor == null) {
                    failures.add("module[" + i + "] drive motor missing");
                } else {
                    int index = i;
                    probes.add(new ConnectivityProbe(
                            "module[" + index + "] drive",
                            () -> driveMotor.isConnected(true),
                            describeMotor("module[" + index + "] drive", driveMotor)));
                }

                MotorController steerMotor = module.getSteerMotorController();
                if (steerMotor == null) {
                    failures.add("module[" + i + "] steer motor missing");
                } else {
                    int index = i;
                    probes.add(new ConnectivityProbe(
                            "module[" + index + "] steer",
                            () -> steerMotor.isConnected(true),
                            describeMotor("module[" + index + "] steer", steerMotor)));
                }

                Encoder steerEncoder = module.getSteerEncoder();
                if (steerEncoder == null) {
                    failures.add("module[" + i + "] steer encoder missing");
                } else {
                    int index = i;
                    probes.add(new ConnectivityProbe(
                            "module[" + index + "] steer encoder",
                            () -> steerEncoder.isConnected(true),
                            describeEncoder("module[" + index + "] steer encoder", steerEncoder)));
                }
            }
        }

        if (probes.isEmpty()) {
            reportConnectivityFailures("Swerve", failures, connectivityFailFastEnabled());
            return;
        }

        boolean failFast = connectivityFailFastEnabled();
        if (connectivityAsyncEnabled() && !failFast) {
            List<String> baseFailures = List.copyOf(failures);
            List<ConnectivityProbe> queuedProbes = List.copyOf(probes);
            Thread asyncCheck = new Thread(
                    () -> {
                        List<String> asyncFailures = new ArrayList<>(baseFailures);
                        asyncFailures.addAll(runConnectivityProbes(queuedProbes, connectivityTimeoutMs()));
                        reportConnectivityFailures("Swerve", asyncFailures, false);
                    },
                    "athena-connectivity-swerve");
            asyncCheck.setDaemon(true);
            asyncCheck.start();
            return;
        }

        failures.addAll(runConnectivityProbes(probes, connectivityTimeoutMs()));
        reportConnectivityFailures("Swerve", failures, failFast);
    }

    private static List<String> runConnectivityProbes(List<ConnectivityProbe> probes, long timeoutMs) {
        if (probes == null || probes.isEmpty()) {
            return List.of();
        }
        int parallelism = resolveConnectivityParallelism(probes.size());
        ExecutorService executor = Executors.newFixedThreadPool(parallelism, connectivityThreadFactory());
        List<Future<Boolean>> futures = new ArrayList<>(probes.size());
        try {
            for (ConnectivityProbe probe : probes) {
                if (probe == null || probe.check() == null) {
                    futures.add(null);
                    continue;
                }
                Callable<Boolean> callable = probe.check()::call;
                futures.add(executor.submit(callable));
            }

            List<String> failures = new ArrayList<>();
            long timeoutNanos = TimeUnit.MILLISECONDS.toNanos(Math.max(1L, timeoutMs));
            long deadlineNs = System.nanoTime() + timeoutNanos;
            for (int i = 0; i < probes.size(); i++) {
                ConnectivityProbe probe = probes.get(i);
                Future<Boolean> future = futures.get(i);
                if (probe == null || future == null) {
                    continue;
                }
                long remainingNs = deadlineNs - System.nanoTime();
                if (remainingNs <= 0L) {
                    future.cancel(true);
                    failures.add(probe.label() + " connectivity probe timed out");
                    continue;
                }
                try {
                    boolean connected = future.get(remainingNs, TimeUnit.NANOSECONDS);
                    if (!connected) {
                        failures.add(probe.failureMessage());
                    }
                } catch (TimeoutException ex) {
                    future.cancel(true);
                    failures.add(probe.label() + " connectivity probe timed out");
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                    failures.add(probe.label() + " connectivity probe interrupted");
                } catch (ExecutionException ex) {
                    Throwable cause = ex.getCause();
                    String message = cause != null && cause.getMessage() != null && !cause.getMessage().isBlank()
                            ? cause.getMessage()
                            : cause != null
                                    ? cause.getClass().getSimpleName()
                                    : ex.getClass().getSimpleName();
                    failures.add(probe.label() + " connectivity probe failed: " + message);
                }
            }
            return failures;
        } finally {
            for (Future<Boolean> future : futures) {
                if (future != null && !future.isDone()) {
                    future.cancel(true);
                }
            }
            executor.shutdownNow();
        }
    }

    private static void reportConnectivityFailures(String drivetrainType, List<String> failures, boolean failFast) {
        if (failures == null || failures.isEmpty()) {
            return;
        }
        String message = drivetrainType
                + " drivetrain required-device connectivity check failed: "
                + String.join("; ", failures);
        if (failFast) {
            DriverStation.reportError(message, false);
            throw new IllegalStateException(message);
        }
        DriverStation.reportWarning(message, false);
    }

    private static boolean connectivityFailFastEnabled() {
        return parseBooleanProperty(CONNECTIVITY_FAIL_FAST_PROPERTY, false);
    }

    private static boolean connectivityAsyncEnabled() {
        return parseBooleanProperty(CONNECTIVITY_ASYNC_PROPERTY, true);
    }

    private static long connectivityTimeoutMs() {
        return parsePositiveLongProperty(CONNECTIVITY_TIMEOUT_MS_PROPERTY, DEFAULT_CONNECTIVITY_TIMEOUT_MS);
    }

    private static int resolveConnectivityParallelism(int probeCount) {
        int fallback = Math.max(1, Math.min(MAX_CONNECTIVITY_CHECK_THREADS, probeCount));
        Integer configured = parsePositiveIntProperty(CONNECTIVITY_PARALLELISM_PROPERTY);
        if (configured == null) {
            return fallback;
        }
        return Math.max(1, Math.min(probeCount, configured));
    }

    private static ThreadFactory connectivityThreadFactory() {
        return runnable -> {
            Thread thread = new Thread(
                    runnable,
                    "athena-connectivity-check-" + CONNECTIVITY_THREAD_COUNTER.getAndIncrement());
            thread.setDaemon(true);
            return thread;
        };
    }

    private static Integer parsePositiveIntProperty(String key) {
        String raw = System.getProperty(key);
        if (raw == null || raw.isBlank()) {
            return null;
        }
        try {
            int parsed = Integer.parseInt(raw.trim());
            return parsed > 0 ? parsed : null;
        } catch (NumberFormatException ignored) {
            return null;
        }
    }

    private static long parsePositiveLongProperty(String key, long fallback) {
        String raw = System.getProperty(key);
        if (raw == null || raw.isBlank()) {
            return fallback;
        }
        try {
            long parsed = Long.parseLong(raw.trim());
            return parsed > 0L ? parsed : fallback;
        } catch (NumberFormatException ignored) {
            return fallback;
        }
    }

    private static boolean parseBooleanProperty(String key, boolean fallback) {
        String raw = System.getProperty(key);
        if (raw == null || raw.isBlank()) {
            return fallback;
        }
        String normalized = raw.trim().toLowerCase(Locale.ROOT);
        if ("true".equals(normalized) || "1".equals(normalized) || "yes".equals(normalized)) {
            return true;
        }
        if ("false".equals(normalized) || "0".equals(normalized) || "no".equals(normalized)) {
            return false;
        }
        return fallback;
    }

    @FunctionalInterface
    private interface ConnectivityCheck {
        boolean call();
    }

    private record ConnectivityProbe(String label, ConnectivityCheck check, String failureMessage) {
    }

    private static String describeMotor(String label, MotorController motor) {
        String type = motor.getType() != null ? motor.getType().getKey() : "unknown";
        return label + " disconnected (type=" + type + ", id=" + motor.getId() + ", canbus=" + motor.getCanbus() + ")";
    }

    private static String describeEncoder(String label, Encoder encoder) {
        EncoderConfig cfg = encoder.getConfig();
        String type = cfg != null && cfg.type() != null ? cfg.type().getKey() : "unknown";
        int id = cfg != null ? cfg.id() : -1;
        String bus = cfg != null ? cfg.canbus() : "";
        return label + " disconnected (type=" + type + ", id=" + id + ", canbus=" + bus + ")";
    }

    private static String describeImu(Imu imuDevice) {
        ImuConfig cfg = imuDevice.getConfig();
        String type = cfg != null && cfg.type() != null ? cfg.type().getKey() : "unknown";
        int id = cfg != null ? cfg.id() : -1;
        String bus = cfg != null ? cfg.canbus() : "";
        return "imu disconnected (type=" + type + ", id=" + id + ", canbus=" + bus + ")";
    }

}
