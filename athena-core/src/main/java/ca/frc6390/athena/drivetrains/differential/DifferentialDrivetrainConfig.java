package ca.frc6390.athena.drivetrains.differential;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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

    /**
     * Creates a baseline configuration using the provided IMU and track width.
     *
     * @param type IMU platform installed on the robot
     * @param inverted true if yaw output should be inverted
     * @param trackWidth distance between left/right wheel centers in meters
     */
    public static DifferentialDrivetrainConfig defualt(AthenaImu type, boolean inverted, double trackWidth){
        return new DifferentialDrivetrainConfig().setIMU(type, inverted).setTrackWidth(trackWidth);
    }

    /**
     * Creates a baseline configuration with a non-inverted IMU.
     *
     * @param type IMU platform installed on the robot
     * @param trackWidth distance between left/right wheel centers in meters
     */
    public static DifferentialDrivetrainConfig defualt(AthenaImu type, double trackWidth){
        return new DifferentialDrivetrainConfig().setIMU(type, false).setTrackWidth(trackWidth);
    }

    /**
     * Declares the IMU used by the drivetrain and whether its heading is inverted.
     */
    public DifferentialDrivetrainConfig setIMU(AthenaImu imu, boolean inverted){
        this.imu = new ImuConfig(imu.resolve(), DrivetrainIDs.DUAL_MOTOR_DIFFERENTIAL.getGyro()).setInverted(inverted);
        return this;
    }

    /**
     * Defines the physical track width (meters).
     */
    public DifferentialDrivetrainConfig setTrackWidth(double trackWidth){
        this.trackWidth = trackWidth;
        return this;
    }

    /**
     * Configures left-side motors using the provided {@link Motor} helper and CAN IDs. Negative IDs
     * invert that motor.
     */
    public DifferentialDrivetrainConfig setLeftMotors(AthenaMotor motor, int... ids){
        return setLeftMotors(Arrays.stream(ids).mapToObj(id -> new MotorControllerConfig(motor.resolveController(), id)).toArray(MotorControllerConfig[]::new));        
    }

    /**
     * Configures right-side motors using the provided {@link Motor} helper and CAN IDs. Negative IDs
     * invert that motor.
     */
    public DifferentialDrivetrainConfig setRightMotors(AthenaMotor motor, int... ids){
        return setRightMotors(Arrays.stream(ids).mapToObj(id -> new MotorControllerConfig(motor.resolveController(), id)).toArray(MotorControllerConfig[]::new));        
    }

    /**
     * Supplies fully constructed motor controller configs for the left side.
     */
    public DifferentialDrivetrainConfig setLeftMotors(MotorControllerConfig... config){
        this.leftMotors = config;
        return this;
    }

    /**
     * Supplies fully constructed motor controller configs for the right side.
     */
    public DifferentialDrivetrainConfig setRightMotors(MotorControllerConfig... config){
        this.rightMotors = config;
        return this;
    }

    /**
     * Configures left encoders using the provided type and hardware IDs. Negative IDs invert the encoder.
     */
    public DifferentialDrivetrainConfig setLeftEncoders(EncoderType encoder, int... ids){
        return setLeftMotors(Arrays.stream(ids).mapToObj(id -> EncoderConfig.type(encoder, id)).toArray(MotorControllerConfig[]::new));        
    }

    /**
     * Configures right encoders using the provided type and hardware IDs. Negative IDs invert the encoder.
     */
    public DifferentialDrivetrainConfig setRightEncoders(EncoderType encoder, int... ids){
        return setRightMotors(Arrays.stream(ids).mapToObj(id -> EncoderConfig.type(encoder, id)).toArray(MotorControllerConfig[]::new));        
    }

    /**
     * Supplies fully constructed encoder configs for the left side.
     */
    public DifferentialDrivetrainConfig setLeftEncoders(EncoderConfig... config){
        this.leftEncoders = config;
        return this;
    }

    /**
     * Supplies fully constructed encoder configs for the right side.
     */
    public DifferentialDrivetrainConfig setRightEncoders(EncoderConfig... config){
        this.rightEncoders = config;
        return this;
    }

    /**
     * Defines the gear ratio from motor rotations to wheel rotations.
     */
    public DifferentialDrivetrainConfig setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
        return this;
    }

    /**
     * Declares the physical wheel diameter used for distance conversions (meters).
     */
    public DifferentialDrivetrainConfig setWheelDiameter(double wheelDiameterMeters){
        this.wheelDiameterMeters = wheelDiameterMeters;
        return this;
    }

    /**
     * Provides drive feedforward gains used to compute voltage setpoints.
     */
    public DifferentialDrivetrainConfig setDriveFeedforward(SimpleMotorFeedforward feedforward) {
        this.driveFeedforward = feedforward;
        return this;
    }

    /**
     * Convenience helper to configure drive feedforward gains.
     */
    public DifferentialDrivetrainConfig setDriveFeedforward(double kS, double kV, double kA) {
        return setDriveFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    /**
     * Enables or disables the configured drive feedforward.
     */
    public DifferentialDrivetrainConfig setDriveFeedforwardEnabled(boolean enabled) {
        this.driveFeedforwardEnabled = enabled;
        return this;
    }

    /**
     * Supplies a custom simulation configuration. Provide {@code null} to disable drivetrain
     * simulation.
     */
    public DifferentialDrivetrainConfig setSimulationConfig(DifferentialSimulationConfig simulationConfig){
        this.simulationConfig = simulationConfig;
        return this;
    }

    // public DifferentialDrivetrainConfig setDriftPID(double kP, double kI, double kd){
    //     return setDriftPID(new PIDController(kP, kI, kd));
    // }
    
    // public DifferentialDrivetrainConfig setDriftPID(PIDController driftPID){
    //     this.driftPID = driftPID;
    //     return this;
    // }

    // public DifferentialDrivetrainConfig setDriftActivationSpeed(double driftActivationSpeed){
    //     this.driftActivationSpeed = driftActivationSpeed;
    //     return this;
    // }

    /**
     * Overrides the drive motor CAN IDs (left followed by right). Negative IDs invert the controller.
     */
    public DifferentialDrivetrainConfig setDriveIds(int[] driveIds){
        this.driveIds = driveIds;
        return this;
    } 

    /**
     * Overrides the encoder IDs (left followed by right). Negative IDs invert the encoder.
     */
    public DifferentialDrivetrainConfig setEncoderIds(int[] encoderIds){
        this.encoderIds = encoderIds;
        return this;
    } 

    /**
     * Populates motor IDs using one of the {@link DriveIDs} presets.
     */
    public DifferentialDrivetrainConfig setDriveIds(DriveIDs driveIds){
       return setDriveIds(driveIds.getIDs());
    } 

    /**
     * Populates encoder IDs using one of the {@link EncoderIDs} presets.
     */
    public DifferentialDrivetrainConfig setEncoderIds(EncoderIDs encoderIds){
        return setEncoderIds(encoderIds.getIDs());
    } 

    /**
     * Sets the CAN ID used by the IMU.
     */
    public DifferentialDrivetrainConfig setIMUId(int id){
        this.imu = this.imu.setId(id);
        return this;
    }

    /**
     * Applies a bundled {@link DrivetrainIDs} preset covering motors, encoders, and gyro.
     */
    public DifferentialDrivetrainConfig setIds(DrivetrainIDs ids){
        setDriveIds(ids.getDrive());
        setEncoderIds(ids.getEncoders());
        setIMUId(ids.getGyro());
        return this;
    } 

    /**
     * Sets whether the drive motors should be inverted.
     */
    public DifferentialDrivetrainConfig setDriveInverted(boolean driveInverted){
        this.driveInverted = driveInverted;
        return this;
    } 
    
    /**
     * Sets whether the encoders should be inverted.
     */
    public DifferentialDrivetrainConfig setEncoderInverted(boolean encoderInverted){
        this.encoderInverted = encoderInverted;
        return this;
    } 

    /**
     * Applies the current limit (amps) to both sides of the drivetrain.
     */
    public DifferentialDrivetrainConfig setCurrentLimit(double currentLimit){
        this.currentLimit = currentLimit;
        return this;
    } 

    /**
     * Sets the CAN bus shared by all drivetrain devices.
     */
    public DifferentialDrivetrainConfig setCanbus(String canbus){
        this.canbus = canbus;
        return this;
    } 

    /**
     * Declares the estimated maximum linear velocity (m/s) used for motion planning.
     */
    public DifferentialDrivetrainConfig setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }

    public DifferentialDrivetrainConfig addSpeedSource(String name, boolean enabledByDefault) {
        speedSources.add(new SpeedSourceRegistration(name, enabledByDefault));
        return this;
    }

    public DifferentialDrivetrainConfig addSpeedBlend(
            String target,
            String source,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        return addSpeedBlend(target, target, source, blendMode, axes);
    }

    public DifferentialDrivetrainConfig addSpeedBlend(
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

    public DifferentialDrivetrainConfig addSpeedOutputBlend(
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
            imu = imu.setCanbus(canbus);
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

        // if (driftPID != null){
        //     dt.setDriftCorrectionPID(driftPID);
        //     dt.setDriftCorrectionMode(true);
        // }

        // dt.setFieldRelative(fieldRelative);
        // dt.driftActivationSpeed = driftActivationSpeed;
        
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
