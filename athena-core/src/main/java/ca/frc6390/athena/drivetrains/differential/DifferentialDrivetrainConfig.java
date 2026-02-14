package ca.frc6390.athena.drivetrains.differential;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
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
public class DifferentialDrivetrainConfig implements RobotDrivetrainConfig<DifferentialDrivetrain>{
   
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
    /** Additional speed sources to register beyond drive/auto/feedback. */
    private final List<SpeedSourceRegistration> speedSources = new ArrayList<>();
    /** Ordered blend rules that write into sources. */
    private final List<SpeedSourceBlendRegistration> speedSourceBlends = new ArrayList<>();
    /** Ordered blend rules that write into the final drivetrain output. */
    private final List<SpeedOutputBlendRegistration> speedOutputBlends = new ArrayList<>();

    private record SpeedSourceRegistration(String name, boolean enabledByDefault) {}

    private record SpeedSourceBlendRegistration(
            String target,
            String left,
            String right,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis[] axes) {}

    private record SpeedOutputBlendRegistration(
            String source,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis[] axes) {}

    public static DifferentialDrivetrainConfig create() {
        return new DifferentialDrivetrainConfig();
    }

    public DifferentialDrivetrainConfig hardware(Consumer<HardwareSection> section) {
        if (section != null) {
            section.accept(new HardwareSection());
        }
        return this;
    }

    public HardwareSection hardware() {
        return new HardwareSection();
    }

    public DifferentialDrivetrainConfig control(Consumer<ControlSection> section) {
        if (section != null) {
            section.accept(new ControlSection());
        }
        return this;
    }

    public ControlSection control() {
        return new ControlSection();
    }

    public DifferentialDrivetrainConfig simulation(Consumer<SimulationSection> section) {
        if (section != null) {
            section.accept(new SimulationSection());
        }
        return this;
    }

    public SimulationSection simulation() {
        return new SimulationSection();
    }

    public DifferentialDrivetrainConfig speed(Consumer<SpeedSection> section) {
        if (section != null) {
            section.accept(new SpeedSection());
        }
        return this;
    }

    public SpeedSection speed() {
        return new SpeedSection();
    }

    public DifferentialDrivetrainConfig config(Consumer<ConfigSection> section) {
        if (section != null) {
            section.accept(new ConfigSection());
        }
        return this;
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
            if (section != null) {
                section.accept(new SpeedSection());
            }
            return this;
        }

        public SpeedSection speed() {
            return new SpeedSection();
        }
    }

    public final class SpeedSection {
        public SpeedSection source(String name, boolean enabledByDefault) {
            addSpeedSource(name, enabledByDefault);
            return this;
        }

        public SpeedSection blend(
                String target,
                String source,
                RobotSpeeds.BlendMode blendMode,
                RobotSpeeds.SpeedAxis... axes) {
            addSpeedBlend(target, source, blendMode, axes);
            return this;
        }

        public SpeedSection blend(
                String target,
                String left,
                String right,
                RobotSpeeds.BlendMode blendMode,
                RobotSpeeds.SpeedAxis... axes) {
            addSpeedBlend(target, left, right, blendMode, axes);
            return this;
        }

        public SpeedSection outputBlend(
                String source,
                RobotSpeeds.BlendMode blendMode,
                RobotSpeeds.SpeedAxis... axes) {
            addSpeedOutputBlend(source, blendMode, axes);
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

    private DifferentialDrivetrainConfig addSpeedSource(String name, boolean enabledByDefault) {
        speedSources.add(new SpeedSourceRegistration(name, enabledByDefault));
        return this;
    }

    private DifferentialDrivetrainConfig addSpeedBlend(
            String target,
            String source,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        return addSpeedBlend(target, target, source, blendMode, axes);
    }

    private DifferentialDrivetrainConfig addSpeedBlend(
            String target,
            String left,
            String right,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        RobotSpeeds.SpeedAxis[] resolvedAxes =
                axes == null ? new RobotSpeeds.SpeedAxis[0] : axes.clone();
        speedSourceBlends.add(new SpeedSourceBlendRegistration(target, left, right, blendMode, resolvedAxes));
        return this;
    }

    private DifferentialDrivetrainConfig addSpeedOutputBlend(
            String source,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        RobotSpeeds.SpeedAxis[] resolvedAxes =
                axes == null ? new RobotSpeeds.SpeedAxis[0] : axes.clone();
        speedOutputBlends.add(new SpeedOutputBlendRegistration(source, blendMode, resolvedAxes));
        return this;
    }

    /**
     * Applies all registered configuration data to the instantiated drivetrain, wiring up motors,
     * encoders, and IMU before returning the fully constructed {@link DifferentialDrivetrain}.
     */
    @Override
    public DifferentialDrivetrain build() {
       
        for (int i = 0; i < leftMotors.length; i++) {
            leftMotors[i] = leftMotors[i].setId(driveIds[i])
                                    .setInverted(driveInverted)
                                    .setCurrentLimit(currentLimit)
                                    .setCanbus(canbus)
                                    .setInverted(driveIds[i] > 0);
        } 

        for (int i = 0; i < rightMotors.length; i++) {
            rightMotors[i] = rightMotors[i].setId(driveIds[i+leftMotors.length])
                                    .setInverted(driveInverted)
                                    .setCurrentLimit(currentLimit)
                                    .setCanbus(canbus)
                                    .setInverted(driveIds[i+leftMotors.length] > 0);
        } 

        if(leftEncoders != null) {
            for (int i = 0; i < leftEncoders.length; i++) {
                leftEncoders[i] = leftEncoders[i].setId(encoderIds[i])
                                        .setInverted(encoderInverted)
                                        .setCanbus(canbus)
                                        .setGearRatio(gearRatio)
                                        .setConversion(Math.PI * wheelDiameterMeters)
                                        .setInverted(encoderIds[i+leftMotors.length] > 0);
            } 
        }

        if(rightEncoders != null) {
            int leftlen = leftEncoders != null ? leftEncoders.length : 0;
            for (int i = 0; i < rightEncoders.length; i++) {
                rightEncoders[i] = rightEncoders[i].setId(encoderIds[i+leftlen])
                                        .setInverted(encoderInverted)
                                        .setCanbus(canbus)
                                        .setGearRatio(gearRatio)
                                        .setConversion(Math.PI *wheelDiameterMeters)
                                        .setInverted(encoderIds[i+leftlen] > 0);
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
            dt.setDriveFeedforward(driveFeedforward);
            dt.setDriveFeedforwardEnabled(driveFeedforwardEnabled);
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

        applySpeedConfig(dt.getRobotSpeeds());
        return dt;
    }

    private void applySpeedConfig(RobotSpeeds speeds) {
        if (speeds == null) {
            return;
        }
        speeds.resetBlendsToDefaults();
        for (SpeedSourceRegistration source : speedSources) {
            speeds.registerSpeedSource(source.name(), source.enabledByDefault());
        }
        for (SpeedSourceBlendRegistration blend : speedSourceBlends) {
            speeds.blend(
                    blend.target(),
                    blend.left(),
                    blend.right(),
                    blend.blendMode(),
                    blend.axes());
        }
        for (SpeedOutputBlendRegistration blend : speedOutputBlends) {
            speeds.blendToOutput(
                    blend.source(),
                    blend.blendMode(),
                    blend.axes());
        }
    }
}
