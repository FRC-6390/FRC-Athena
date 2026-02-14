package ca.frc6390.athena.drivetrains.differential;

import java.util.Arrays;
import java.util.function.Consumer;

import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.drivetrains.DrivetrainSpeedSectionBase;
import ca.frc6390.athena.drivetrains.DrivetrainSpeedConfigSupport;
import ca.frc6390.athena.drivetrains.SectionedDrivetrainConfig;
import ca.frc6390.athena.core.sections.SectionedAccess;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DriveIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.EncoderIDs;
import ca.frc6390.athena.hardware.imu.AthenaImu;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.encoder.EncoderGroup;
import ca.frc6390.athena.hardware.imu.VirtualImu;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.drivetrains.differential.sim.DifferentialSimulationConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Builder for {@link DifferentialDrivetrain} instances. Captures the motor/encoder layout, CAN IDs,
 * inversion flags, gearing, and current limits necessary to initialize the drivetrain consistently.
 */
public class DifferentialDrivetrainConfig extends SectionedDrivetrainConfig<DifferentialDrivetrainConfig>
        implements RobotDrivetrainConfig<DifferentialDrivetrain>{
   
    /** Estimated maximum linear velocity of the drivetrain (m/s). */
    private double maxVelocity;
    /** Track width (meters) used for odometry integration and curvature feedforward. */
    private double trackWidth;
    //DEFUALT VALUES
    /** IMU configuration used to provide heading. */
    private ImuConfig imu = null;
    /** Motor configurations for the left drivetrain side. */
    private MotorControllerConfig[] leftMotors, rightMotors;
    /** Encoder configurations for the left and right drivetrain sides. */
    private EncoderConfig[] leftEncoders, rightEncoders;
    // private PIDController driftPID = null;
    // private double driftActivationSpeed = 0.05;
    /** Default drive motor CAN IDs (negatives invert direction). */
    private int[] driveIds = DriveIDs.DUAL_MOTOR_DIFFERENTIAL.getIDs();
    /** Default encoder CAN/DIO IDs (negatives invert direction). */
    private int[] encoderIds = EncoderIDs.DUAL_MOTOR_DIFFERENTIAL.getIDs();
    /** Whether encoders should be inverted when configured. */
    private boolean encoderInverted = false;
    /** Whether drive motors should be inverted when configured. */
    private boolean driveInverted = false;
    /** Current limit applied to each drive motor (amps). */
    private double currentLimit = 80;
    /** CAN bus shared by all drivetrain devices. */
    private String canbus = "rio";
    /** Gear ratio from motor to wheel. */
    private double gearRatio = 1;
    /** Physical wheel diameter used for distance conversions (meters). */
    private double wheelDiameterMeters = 1;
    /** Optional physics model used when running drivetrain simulation. */
    private DifferentialSimulationConfig simulationConfig = DifferentialSimulationConfig.defaults();
    /** Optional drive feedforward used to command wheel velocities as voltages. */
    private SimpleMotorFeedforward driveFeedforward = null;
    /** Whether the configured drive feedforward is enabled. */
    private boolean driveFeedforwardEnabled = true;
    /** Shared speed source/blend registration store used by speed() sections. */
    private final DrivetrainSpeedConfigSupport speedConfig = new DrivetrainSpeedConfigSupport();

    public static DifferentialDrivetrainConfig create() {
        return new DifferentialDrivetrainConfig();
    }

    @Override
    protected DifferentialDrivetrainConfig self() {
        return this;
    }

    public DifferentialDrivetrainConfig hardware(Consumer<HardwareSection> section) {
        return applySection(section, HardwareSection::new);
    }

    public HardwareSection hardware() {
        return new HardwareSection();
    }

    public DifferentialDrivetrainConfig control(Consumer<ControlSection> section) {
        return applySection(section, ControlSection::new);
    }

    public ControlSection control() {
        return new ControlSection();
    }

    public DifferentialDrivetrainConfig simulation(Consumer<SimulationSection> section) {
        return applySection(section, SimulationSection::new);
    }

    public SimulationSection simulation() {
        return new SimulationSection();
    }

    public DifferentialDrivetrainConfig speed(Consumer<SpeedSection> section) {
        return applySection(section, SpeedSection::new);
    }

    public SpeedSection speed() {
        return new SpeedSection();
    }

    public DifferentialDrivetrainConfig config(Consumer<ConfigSection> section) {
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

        public HardwareSection trackWidth(double meters) {
            applyTrackWidth(meters);
            return this;
        }

        public HardwareSection leftMotors(AthenaMotor motor, int... ids) {
            applyLeftMotors(motor, ids);
            return this;
        }

        public HardwareSection rightMotors(AthenaMotor motor, int... ids) {
            applyRightMotors(motor, ids);
            return this;
        }

        public HardwareSection leftMotors(MotorControllerConfig... configs) {
            applyLeftMotors(configs);
            return this;
        }

        public HardwareSection rightMotors(MotorControllerConfig... configs) {
            applyRightMotors(configs);
            return this;
        }

        public HardwareSection leftEncoders(EncoderType encoderType, int... ids) {
            applyLeftEncoders(encoderType, ids);
            return this;
        }

        public HardwareSection rightEncoders(EncoderType encoderType, int... ids) {
            applyRightEncoders(encoderType, ids);
            return this;
        }

        public HardwareSection leftEncoders(EncoderConfig... configs) {
            applyLeftEncoders(configs);
            return this;
        }

        public HardwareSection rightEncoders(EncoderConfig... configs) {
            applyRightEncoders(configs);
            return this;
        }

        public HardwareSection gearRatio(double ratio) {
            applyGearRatio(ratio);
            return this;
        }

        public HardwareSection wheelDiameter(double meters) {
            applyWheelDiameter(meters);
            return this;
        }

        public HardwareSection driveIds(int[] ids) {
            applyDriveIds(ids);
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

        public HardwareSection encoderInverted(boolean inverted) {
            applyEncoderInverted(inverted);
            return this;
        }

        public HardwareSection currentLimit(double amps) {
            applyCurrentLimit(amps);
            return this;
        }

        public HardwareSection canbus(String bus) {
            applyCanbus(bus);
            return this;
        }

        public HardwareSection maxVelocity(double metersPerSecond) {
            applyMaxVelocity(metersPerSecond);
            return this;
        }
    }

    public final class ControlSection {
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
    }

    public final class SimulationSection {
        public SimulationSection config(DifferentialSimulationConfig simulation) {
            applySimulationConfig(simulation);
            return this;
        }
    }

    public final class ConfigSection {
        public ConfigSection imu(AthenaImu imuType, boolean inverted) {
            applyImu(imuType, inverted);
            return this;
        }

        public ConfigSection trackWidth(double meters) {
            applyTrackWidth(meters);
            return this;
        }

        public ConfigSection leftMotors(AthenaMotor motor, int... ids) {
            applyLeftMotors(motor, ids);
            return this;
        }

        public ConfigSection rightMotors(AthenaMotor motor, int... ids) {
            applyRightMotors(motor, ids);
            return this;
        }

        public ConfigSection leftMotors(MotorControllerConfig... configs) {
            applyLeftMotors(configs);
            return this;
        }

        public ConfigSection rightMotors(MotorControllerConfig... configs) {
            applyRightMotors(configs);
            return this;
        }

        public ConfigSection leftEncoders(EncoderType encoderType, int... ids) {
            applyLeftEncoders(encoderType, ids);
            return this;
        }

        public ConfigSection rightEncoders(EncoderType encoderType, int... ids) {
            applyRightEncoders(encoderType, ids);
            return this;
        }

        public ConfigSection leftEncoders(EncoderConfig... configs) {
            applyLeftEncoders(configs);
            return this;
        }

        public ConfigSection rightEncoders(EncoderConfig... configs) {
            applyRightEncoders(configs);
            return this;
        }

        public ConfigSection gearRatio(double ratio) {
            applyGearRatio(ratio);
            return this;
        }

        public ConfigSection wheelDiameter(double meters) {
            applyWheelDiameter(meters);
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

        public ConfigSection simulationConfig(DifferentialSimulationConfig simulation) {
            applySimulationConfig(simulation);
            return this;
        }

        public ConfigSection driveIds(int[] ids) {
            applyDriveIds(ids);
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

        public ConfigSection encoderInverted(boolean inverted) {
            applyEncoderInverted(inverted);
            return this;
        }

        public ConfigSection currentLimit(double amps) {
            applyCurrentLimit(amps);
            return this;
        }

        public ConfigSection canbus(String bus) {
            applyCanbus(bus);
            return this;
        }

        public ConfigSection maxVelocity(double metersPerSecond) {
            applyMaxVelocity(metersPerSecond);
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
     * Creates a baseline configuration using the provided IMU and track width.
     *
     * @param type IMU platform installed on the robot
     * @param inverted true if yaw output should be inverted
     * @param trackWidth distance between left/right wheel centers in meters
     */
    public static DifferentialDrivetrainConfig defaults(AthenaImu type, boolean inverted, double trackWidth){
        return create().hardware(h -> h.imu(type, inverted).trackWidth(trackWidth));
    }

    /**
     * Creates a baseline configuration with a non-inverted IMU.
     *
     * @param type IMU platform installed on the robot
     * @param trackWidth distance between left/right wheel centers in meters
     */
    public static DifferentialDrivetrainConfig defaults(AthenaImu type, double trackWidth){
        return create().hardware(h -> h.imu(type, false).trackWidth(trackWidth));
    }

    /**
     * Declares the IMU used by the drivetrain and whether its heading is inverted.
     */
    private DifferentialDrivetrainConfig applyImu(AthenaImu imu, boolean inverted){
        this.imu = ImuConfig
                .create(imu.resolve(), DrivetrainIDs.DUAL_MOTOR_DIFFERENTIAL.getGyro())
                .hardware(h -> h.inverted(inverted));
        return this;
    }

    /**
     * Defines the physical track width (meters).
     */
    private DifferentialDrivetrainConfig applyTrackWidth(double trackWidth){
        this.trackWidth = trackWidth;
        return this;
    }

    /**
     * Configures left-side motors using the provided {@link Motor} helper and CAN IDs. Negative IDs
     * invert that motor.
     */
    private DifferentialDrivetrainConfig applyLeftMotors(AthenaMotor motor, int... ids){
        return applyLeftMotors(Arrays.stream(ids)
                .mapToObj(id -> MotorControllerConfig.create(motor.resolveController(), id))
                .toArray(MotorControllerConfig[]::new));
    }

    /**
     * Configures right-side motors using the provided {@link Motor} helper and CAN IDs. Negative IDs
     * invert that motor.
     */
    private DifferentialDrivetrainConfig applyRightMotors(AthenaMotor motor, int... ids){
        return applyRightMotors(Arrays.stream(ids)
                .mapToObj(id -> MotorControllerConfig.create(motor.resolveController(), id))
                .toArray(MotorControllerConfig[]::new));
    }

    /**
     * Supplies fully constructed motor controller configs for the left side.
     */
    private DifferentialDrivetrainConfig applyLeftMotors(MotorControllerConfig... config){
        this.leftMotors = config;
        return this;
    }

    /**
     * Supplies fully constructed motor controller configs for the right side.
     */
    private DifferentialDrivetrainConfig applyRightMotors(MotorControllerConfig... config){
        this.rightMotors = config;
        return this;
    }

    /**
     * Configures left encoders using the provided type and hardware IDs. Negative IDs invert the encoder.
     */
    private DifferentialDrivetrainConfig applyLeftEncoders(EncoderType encoder, int... ids){
        return applyLeftEncoders(Arrays.stream(ids)
                .mapToObj(id -> EncoderConfig.create(encoder, id))
                .toArray(EncoderConfig[]::new));
    }

    /**
     * Configures right encoders using the provided type and hardware IDs. Negative IDs invert the encoder.
     */
    private DifferentialDrivetrainConfig applyRightEncoders(EncoderType encoder, int... ids){
        return applyRightEncoders(Arrays.stream(ids)
                .mapToObj(id -> EncoderConfig.create(encoder, id))
                .toArray(EncoderConfig[]::new));
    }

    /**
     * Supplies fully constructed encoder configs for the left side.
     */
    private DifferentialDrivetrainConfig applyLeftEncoders(EncoderConfig... config){
        this.leftEncoders = config;
        return this;
    }

    /**
     * Supplies fully constructed encoder configs for the right side.
     */
    private DifferentialDrivetrainConfig applyRightEncoders(EncoderConfig... config){
        this.rightEncoders = config;
        return this;
    }

    /**
     * Defines the gear ratio from motor rotations to wheel rotations.
     */
    private DifferentialDrivetrainConfig applyGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
        return this;
    }

    /**
     * Declares the physical wheel diameter used for distance conversions (meters).
     */
    private DifferentialDrivetrainConfig applyWheelDiameter(double wheelDiameterMeters){
        this.wheelDiameterMeters = wheelDiameterMeters;
        return this;
    }

    /**
     * Provides drive feedforward gains used to compute voltage setpoints.
     */
    private DifferentialDrivetrainConfig applyDriveFeedforward(SimpleMotorFeedforward feedforward) {
        this.driveFeedforward = feedforward;
        return this;
    }

    /**
     * Convenience helper to configure drive feedforward gains.
     */
    private DifferentialDrivetrainConfig applyDriveFeedforward(double kS, double kV, double kA) {
        return applyDriveFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    /**
     * Enables or disables the configured drive feedforward.
     */
    private DifferentialDrivetrainConfig applyDriveFeedforwardEnabled(boolean enabled) {
        this.driveFeedforwardEnabled = enabled;
        return this;
    }

    /**
     * Supplies a custom simulation configuration. Provide {@code null} to disable drivetrain
     * simulation.
     */
    private DifferentialDrivetrainConfig applySimulationConfig(DifferentialSimulationConfig simulationConfig){
        this.simulationConfig = simulationConfig;
        return this;
    }

    /**
     * Overrides the drive motor CAN IDs (left followed by right). Negative IDs invert the controller.
     */
    private DifferentialDrivetrainConfig applyDriveIds(int[] driveIds){
        this.driveIds = driveIds;
        return this;
    } 

    /**
     * Overrides the encoder IDs (left followed by right). Negative IDs invert the encoder.
     */
    private DifferentialDrivetrainConfig applyEncoderIds(int[] encoderIds){
        this.encoderIds = encoderIds;
        return this;
    } 

    /**
     * Populates motor IDs using one of the {@link DriveIDs} presets.
     */
    private DifferentialDrivetrainConfig applyDriveIds(DriveIDs driveIds){
       return applyDriveIds(driveIds.getIDs());
    } 

    /**
     * Populates encoder IDs using one of the {@link EncoderIDs} presets.
     */
    private DifferentialDrivetrainConfig applyEncoderIds(EncoderIDs encoderIds){
        return applyEncoderIds(encoderIds.getIDs());
    } 

    /**
     * Sets the CAN ID used by the IMU.
     */
    private DifferentialDrivetrainConfig applyImuId(int id){
        this.imu = this.imu.hardware(h -> h.id(id));
        return this;
    }

    /**
     * Applies a bundled {@link DrivetrainIDs} preset covering motors, encoders, and gyro.
     */
    private DifferentialDrivetrainConfig applyIds(DrivetrainIDs ids){
        applyDriveIds(ids.getDrive());
        applyEncoderIds(ids.getEncoders());
        applyImuId(ids.getGyro());
        return this;
    } 

    /**
     * Sets whether the drive motors should be inverted.
     */
    private DifferentialDrivetrainConfig applyDriveInverted(boolean driveInverted){
        this.driveInverted = driveInverted;
        return this;
    } 
    
    /**
     * Sets whether the encoders should be inverted.
     */
    private DifferentialDrivetrainConfig applyEncoderInverted(boolean encoderInverted){
        this.encoderInverted = encoderInverted;
        return this;
    } 

    /**
     * Applies the current limit (amps) to both sides of the drivetrain.
     */
    private DifferentialDrivetrainConfig applyCurrentLimit(double currentLimit){
        this.currentLimit = currentLimit;
        return this;
    } 

    /**
     * Sets the CAN bus shared by all drivetrain devices.
     */
    private DifferentialDrivetrainConfig applyCanbus(String canbus){
        this.canbus = canbus;
        return this;
    } 

    /**
     * Declares the estimated maximum linear velocity (m/s) used for motion planning.
     */
    private DifferentialDrivetrainConfig applyMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }

    /**
     * Applies all registered configuration data to the instantiated drivetrain, wiring up motors,
     * encoders, and IMU before returning the fully constructed {@link DifferentialDrivetrain}.
     */
    @Override
    public DifferentialDrivetrain build() {
       
        for (int i = 0; i < leftMotors.length; i++) {
            leftMotors[i].hardware()
                    .id(driveIds[i])
                    .inverted(driveInverted)
                    .currentLimit(currentLimit)
                    .canbus(canbus)
                    .inverted(driveIds[i] > 0);
        } 

        for (int i = 0; i < rightMotors.length; i++) {
            rightMotors[i].hardware()
                    .id(driveIds[i + leftMotors.length])
                    .inverted(driveInverted)
                    .currentLimit(currentLimit)
                    .canbus(canbus)
                    .inverted(driveIds[i + leftMotors.length] > 0);
        } 

        if(leftEncoders != null) {
            for (int i = 0; i < leftEncoders.length; i++) {
                leftEncoders[i].hardware()
                        .id(encoderIds[i])
                        .inverted(encoderInverted)
                        .canbus(canbus)
                        .inverted(encoderIds[i + leftMotors.length] > 0);
                leftEncoders[i].measurement()
                        .gearRatio(gearRatio)
                        .conversion(Math.PI * wheelDiameterMeters);
            } 
        }

        if(rightEncoders != null) {
            int leftlen = leftEncoders != null ? leftEncoders.length : 0;
            for (int i = 0; i < rightEncoders.length; i++) {
                rightEncoders[i].hardware()
                        .id(encoderIds[i + leftlen])
                        .inverted(encoderInverted)
                        .canbus(canbus)
                        .inverted(encoderIds[i + leftlen] > 0);
                rightEncoders[i].measurement()
                        .gearRatio(gearRatio)
                        .conversion(Math.PI * wheelDiameterMeters);
            } 
        }

        if (leftMotors == null || rightMotors == null) {
            throw new Error("Both left and right motor configurations must be provided.");
        }

        if (imu != null) {
            imu = imu.hardware(h -> h.canbus(canbus));
        }

        MotorController[] leftControllers =
                Arrays.stream(leftMotors).map(HardwareFactories::motor).toArray(MotorController[]::new);
        MotorController[] rightControllers =
                Arrays.stream(rightMotors).map(HardwareFactories::motor).toArray(MotorController[]::new);

        MotorControllerGroup lm = new MotorControllerGroup(leftControllers);
        if (leftEncoders != null) {
            lm.setEncoders(EncoderGroup.fromConfigs(leftEncoders));
        }

        MotorControllerGroup rm = new MotorControllerGroup(rightControllers);
        if (rightEncoders != null) {
            rm.setEncoders(EncoderGroup.fromConfigs(rightEncoders));
        }

        Imu resolvedImu = imu == null ? null : new VirtualImu(HardwareFactories.imu(imu));
        DifferentialDrivetrain dt = new DifferentialDrivetrain(resolvedImu, maxVelocity, trackWidth, lm, rm);

        if (driveFeedforward != null) {
            dt.driveFeedforward(driveFeedforward);
            dt.driveFeedforwardEnabled(driveFeedforwardEnabled);
        }

        if (simulationConfig != null) {
            double simulationGearRatio = gearRatio;
            if (simulationGearRatio > 0 && simulationGearRatio < 1.0) {
                simulationGearRatio = 1.0 / simulationGearRatio;
            }
            DifferentialSimulationConfig resolvedSimulation = simulationConfig.resolve(
                    trackWidth,
                    simulationGearRatio,
                    wheelDiameterMeters,
                    leftMotors != null ? leftMotors.length : 1);
            dt.configureSimulation(resolvedSimulation);
        }

        speedConfig.apply(dt.robotSpeeds());
        return dt;
    }
}
