package ca.frc6390.athena.drivetrains.swerve;

import java.util.function.Consumer;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.sim.MotorSimType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
public class SwerveModule implements RobotSendableDevice {
    
    private final SwerveModuleConfig config;
    private final MotorController driveMotor;
    private final MotorController rotationMotor;
    private final Encoder encoder;
    private final Encoder driveEncoder;
    private final Encoder steerEncoder;
    private final PIDController rotationPidController;
    private final double startUpOffset;
    private double lastDriveCommand = 0;
    private double lastSteerCommand = 0;
    private SimpleMotorFeedforward driveFeedforward;
    private boolean driveFeedforwardEnabled = false;
    private double nominalVoltage = 12.0;
    private double lastSetpointSpeedMetersPerSecond = 0.0;
    private boolean hasPreviousSetpoint = false;
    private ca.frc6390.athena.drivetrains.swerve.sim.SwerveModuleSimulation simulation;
    private final SwerveModuleState stateView = new SwerveModuleState(0.0, Rotation2d.kZero);
    private final SwerveModulePosition positionView = new SwerveModulePosition(0.0, Rotation2d.kZero);
    // SWERVE MOTOR RECORD
    public record SwerveModuleConfig(
            Translation2d module_location,
            double wheelDiameter,
            double maxSpeedMetersPerSecond,
            MotorControllerConfig driveMotor,
            MotorControllerConfig rotationMotor,
            PIDController rotationPID,
            EncoderConfig encoder,
            SwerveModuleSimConfig sim,
            SimpleMotorFeedforward driveFeedforward,
            boolean driveFeedforwardEnabled) {
        public SwerveModuleConfig {
            if (driveFeedforward == null) {
                driveFeedforwardEnabled = false;
            }
        }

        public SwerveModuleConfig(
                Translation2d module_location,
                double wheelDiameter,
                double maxSpeedMetersPerSecond,
                MotorControllerConfig driveMotor,
                MotorControllerConfig rotationMotor,
                PIDController rotationPID,
                EncoderConfig encoder,
                SwerveModuleSimConfig sim) {
            this(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    null,
                    false);
        }

        public SwerveModuleConfig(Translation2d module_location, double wheelDiameter, double maxSpeedMetersPerSecond, MotorControllerConfig driveMotor, MotorControllerConfig rotationMotor, PIDController rotationPID, EncoderConfig encoder) {
            this(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    SwerveModuleSimConfig.fromMotors(null, null),
                    null,
                    false);
        }

        public SwerveModuleConfig(Translation2d module_location, double wheelDiameter, double maxSpeedMetersPerSecond, MotorControllerConfig driveMotor, MotorControllerConfig rotationMotor, PIDController rotationPID) {
            this(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    rotationMotor.encoderConfig(),
                    SwerveModuleSimConfig.fromMotors(null, null),
                    null,
                    false);
        }

        //cloning
        public SwerveModuleConfig(SwerveModuleConfig other) {
            this(
                new Translation2d(other.module_location().getX(), other.module_location().getY()),
                other.wheelDiameter(),
                other.maxSpeedMetersPerSecond(),
                other.driveMotor(),
                other.rotationMotor(),
                clonePid(other.rotationPID()),
                other.encoder(),
                other.sim(),
                cloneFeedforward(other.driveFeedforward()),
                other.driveFeedforwardEnabled()
            );
        }

        private static PIDController clonePid(PIDController source) {
            if (source == null) {
                return null;
            }
            return new PIDController(source.getP(), source.getI(), source.getD());
        }

        private static SimpleMotorFeedforward cloneFeedforward(SimpleMotorFeedforward source) {
            if (source == null) {
                return null;
            }
            return new SimpleMotorFeedforward(source.getKs(), source.getKv(), source.getKa());
        }

        public SwerveModuleConfig canbus(String canbus){
            driveMotor.hardware().canbus(canbus);
            rotationMotor.hardware().canbus(canbus);
            if (encoder != null) {
                encoder.hardware().canbus(canbus);
            }
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled);
        }

        public SwerveModuleConfig pid(Consumer<PidSection> section) {
            PidSection resolved = PidSection.from(rotationPID);
            if (section != null) {
                section.accept(resolved);
            }
            return withRotationPid(resolved.build());
        }

        private SwerveModuleConfig withRotationPid(PIDController rotationPID){
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled);
        }

        public SwerveModuleConfig ff(Consumer<FeedforwardSection> section) {
            FeedforwardSection resolved = FeedforwardSection.from(driveFeedforward, driveFeedforwardEnabled);
            if (section != null) {
                section.accept(resolved);
            }
            return ff(resolved.build(), resolved.isEnabled());
        }

        public SwerveModuleConfig ffEnabled(boolean enabled) {
            return ff(driveFeedforward, enabled);
        }

        private SwerveModuleConfig ff(SimpleMotorFeedforward feedforward, boolean enabled) {
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    feedforward,
                    feedforward != null && enabled);
        }

        public SwerveModuleConfig encoder(EncoderConfig encoder){
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled);
        }

        public SwerveModuleConfig encoder(EncoderType encoder){
            EncoderConfig cfg = encoder() != null ? encoder() : EncoderConfig.create(encoder, 0);
            cfg.hardware().type(encoder);
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    cfg,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled);
        }

        public SwerveModuleConfig offset(double offset){
            if (encoder != null) {
                encoder.measurement().offset(offset);
            }
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled);
        }

        public SwerveModuleConfig location(Translation2d module_location){
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled);
        }

        public SwerveModuleConfig driveId(int id){
            driveMotor.hardware().id(id);
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled); 
        }

        public SwerveModuleConfig steerId(int id){
            rotationMotor.hardware().id(id);
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled); 
        }

        public SwerveModuleConfig encoderId(int id){
            if (encoder != null) {
                encoder.hardware().id(id);
            }
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled); 
        }

        public SwerveModuleConfig driveInverted(boolean inverted){
            driveMotor.hardware().inverted(inverted);
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled); 
        }

        public SwerveModuleConfig steerInverted(boolean inverted){
            rotationMotor.hardware().inverted(inverted);
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled); 
        }

        public SwerveModuleConfig encoderInverted(boolean inverted){
            if (encoder != null) {
                encoder.hardware().inverted(inverted);
            }
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled); 
        }

        public SwerveModuleConfig currentLimit(double currentLimit) {
            return driveCurrentLimit(currentLimit).steerCurrentLimit(currentLimit);
        }

        public SwerveModuleConfig driveCurrentLimit(double currentLimit) {
            driveMotor.hardware().currentLimit(currentLimit);
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled);
        }

        public SwerveModuleConfig steerCurrentLimit(double currentLimit) {
            rotationMotor.hardware().currentLimit(currentLimit);
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled);
        }

        public static Translation2d[] generateModuleLocations(double trackwidth, double wheelbase) {
            // WPILib chassis convention: +X forward (wheelbase), +Y left (trackwidth).
            Translation2d FRONT_LEFT = new Translation2d(wheelbase/2, trackwidth/2);
            Translation2d FRONT_RIGHT = new Translation2d(wheelbase/2, -trackwidth/2);
            Translation2d BACK_LEFT = new Translation2d(-wheelbase/2, trackwidth/2);
            Translation2d BACK_RIGHT = new Translation2d(-wheelbase/2, -trackwidth/2);
            return new Translation2d[]{FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
        }

        public SwerveModuleConfig simulation(SwerveModuleSimConfig sim) {
            return new SwerveModuleConfig(
                    module_location,
                    wheelDiameter,
                    maxSpeedMetersPerSecond,
                    driveMotor,
                    rotationMotor,
                    rotationPID,
                    encoder,
                    sim,
                    driveFeedforward,
                    driveFeedforwardEnabled);
        }

        public static final class PidSection {
            private double kP;
            private double kI;
            private double kD;

            private PidSection(PIDController current) {
                if (current != null) {
                    kP = current.getP();
                    kI = current.getI();
                    kD = current.getD();
                }
            }

            static PidSection from(PIDController current) {
                return new PidSection(current);
            }

            public PidSection p(double kP) {
                this.kP = kP;
                return this;
            }

            public PidSection i(double kI) {
                this.kI = kI;
                return this;
            }

            public PidSection d(double kD) {
                this.kD = kD;
                return this;
            }

            private PIDController build() {
                return new PIDController(kP, kI, kD);
            }
        }

        public static final class FeedforwardSection {
            private double kS;
            private double kV;
            private double kA;
            private boolean enabled;
            private boolean configured;

            private FeedforwardSection(SimpleMotorFeedforward current, boolean enabled) {
                if (current != null) {
                    kS = current.getKs();
                    kV = current.getKv();
                    kA = current.getKa();
                    configured = true;
                }
                this.enabled = current != null && enabled;
            }

            static FeedforwardSection from(SimpleMotorFeedforward current, boolean enabled) {
                return new FeedforwardSection(current, enabled);
            }

            public FeedforwardSection ks(double kS) {
                this.kS = kS;
                if (!configured) {
                    enabled = true;
                }
                configured = true;
                return this;
            }

            public FeedforwardSection kv(double kV) {
                this.kV = kV;
                if (!configured) {
                    enabled = true;
                }
                configured = true;
                return this;
            }

            public FeedforwardSection ka(double kA) {
                this.kA = kA;
                if (!configured) {
                    enabled = true;
                }
                configured = true;
                return this;
            }

            public FeedforwardSection enabled(boolean enabled) {
                this.enabled = enabled;
                return this;
            }

            private SimpleMotorFeedforward build() {
                if (!configured) {
                    return null;
                }
                return new SimpleMotorFeedforward(kS, kV, kA);
            }

            private boolean isEnabled() {
                return enabled;
            }
        }
    }

    public record SwerveModuleSimConfig(MotorSimType driveMotorType, MotorSimType steerMotorType, double driveMomentOfInertia, double steerMomentOfInertia) {

        public static final double DEFAULT_DRIVE_MOI = 0.025;
        public static final double DEFAULT_STEER_MOI = 0.004;

        // Drive and steer inertia values come from SysId identification or CAD; start with defaults and tune until acceleration matches logs.
        public SwerveModuleSimConfig withDriveMotor(MotorSimType drive) {
            return new SwerveModuleSimConfig(drive, steerMotorType, driveMomentOfInertia, steerMomentOfInertia);
        }

        public SwerveModuleSimConfig withSteerMotor(MotorSimType steer) {
            return new SwerveModuleSimConfig(driveMotorType, steer, driveMomentOfInertia, steerMomentOfInertia);
        }

        public SwerveModuleSimConfig withDriveMoment(double moi) {
            return new SwerveModuleSimConfig(driveMotorType, steerMotorType, moi, steerMomentOfInertia);
        }

        public SwerveModuleSimConfig withSteerMoment(double moi) {
            return new SwerveModuleSimConfig(driveMotorType, steerMotorType, driveMomentOfInertia, moi);
        }

        public static SwerveModuleSimConfig fromMotors(MotorSimType drive, MotorSimType steer) {
            return new SwerveModuleSimConfig(drive, steer, DEFAULT_DRIVE_MOI, DEFAULT_STEER_MOI);
        }
    }

    public SwerveModule(SwerveModuleConfig config) {
        this.config = config;
        // MODULE ROTATION PID
        rotationPidController = config.rotationPID();

        // ROTATION PID ENABLING -180 TO 180
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

        // MOTOR INITIALIZATION
        driveMotor = HardwareFactories.motor(config.driveMotor());

        rotationMotor = HardwareFactories.motor(config.rotationMotor());

        encoder = HardwareFactories.encoder(config.encoder());
        driveEncoder = driveMotor.getEncoder();
        steerEncoder = rotationMotor.getEncoder();

        resetEncoders();
        setNeutralMode(MotorNeutralMode.Brake);
        this.startUpOffset = encoder.getOffset();
        if (config.driveFeedforward() != null) {
            this.driveFeedforward = new SimpleMotorFeedforward(
                    config.driveFeedforward().getKs(),
                    config.driveFeedforward().getKv(),
                    config.driveFeedforward().getKa());
            this.driveFeedforwardEnabled = config.driveFeedforwardEnabled();
        }
    }

    public double getDriveMotorVelocity() {
        return driveEncoder != null ? driveEncoder.getVelocity() : 0.0;
    }

    public Translation2d getModuleLocation() {
        return config.module_location();
    }

    public double getDriveMotorPosition() {
        return driveEncoder != null ? driveEncoder.getPosition() : 0.0;
    }

    public Rotation2d getEncoderPosition() {
        return encoder != null ? encoder.getAbsoluteRotation2d() : Rotation2d.kZero;
    }

    public MotorController getDriveMotorController() {
        return driveMotor;
    }

    public MotorController getSteerMotorController() {
        return rotationMotor;
    }

    public Encoder getSteerEncoder() {
        return encoder;
    }

    public SwerveModuleConfig getConfigData() {
        return config;
    }

    public void setOffset(double offset) {
        encoder.setOffset(offset);
    }

    public void resetEncoders() {
        if (driveEncoder != null) {
            driveEncoder.setPosition(0);
        } else {
            DriverStation.reportWarning("Drive encoder not configured for swerve module; position reset skipped.", false);
        }
        if (steerEncoder != null) {
            steerEncoder.setPosition(encoder != null ? encoder.getAbsolutePosition() : 0.0);
        } else {
            DriverStation.reportWarning("Steer encoder not configured for swerve module; position reset skipped.", false);
        }
    }

    public SwerveModuleState getState() {
        stateView.speedMetersPerSecond = getDriveMotorVelocity();
        stateView.angle = getEncoderPosition();
        return stateView;
    }

    public SwerveModulePosition getPostion() {
        positionView.distanceMeters = getDriveMotorPosition();
        positionView.angle = getEncoderPosition();
        return positionView;
    }

    public void setDriveMotor(double speed) {
        lastDriveCommand = speed;
        driveMotor.setSpeed(speed);
    }

    public void setDriveVoltage(double volts) {
        setDriveMotorVoltage(volts);
    }

    private void setDriveMotorVoltage(double volts) {
        double voltageLimit = getVoltageLimit();
        double clamped = MathUtil.clamp(volts, -voltageLimit, voltageLimit);
        lastDriveCommand = MathUtil.clamp(clamped / voltageLimit, -1.0, 1.0);
        driveMotor.setVoltage(clamped);
    }

    public void setRotationMotor(double speed) {
        lastSteerCommand = speed;
        rotationMotor.setSpeed(speed);
    }

    public void setSteerAngle(Rotation2d angle) {
        setRotationMotor(
                rotationPidController.calculate(
                        MathUtil.angleModulus(getEncoderPosition().getRadians()),
                        angle.getRadians()));
    }

    public double getDriveCommandPercent() {
        return lastDriveCommand;
    }

    public double getSteerCommandPercent() {
        return lastSteerCommand;
    }

    public void setDriveFeedforward(SimpleMotorFeedforward feedforward) {
        this.driveFeedforward = feedforward;
        this.driveFeedforwardEnabled = feedforward != null;
    }

    public void setDriveFeedforwardEnabled(boolean enabled) {
        if (driveFeedforward == null) {
            driveFeedforwardEnabled = false;
            return;
        }
        driveFeedforwardEnabled = enabled;
    }

    public boolean isDriveFeedforwardEnabled() {
        return driveFeedforwardEnabled;
    }

    public boolean hasDriveFeedforward() {
        return driveFeedforward != null;
    }

    public void setNominalVoltage(double nominalVoltage) {
        if (Double.isFinite(nominalVoltage) && nominalVoltage > 1e-3) {
            this.nominalVoltage = nominalVoltage;
        }
    }

    public void setDesiredState(SwerveModuleState state) {
        refresh();

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();

            return;
        }

        SwerveModuleState optimizedState =
                new SwerveModuleState(state.speedMetersPerSecond, state.angle);
        Rotation2d currentAngle = getEncoderPosition();
        optimizedState.optimize(currentAngle);
        optimizedState.speedMetersPerSecond *= optimizedState.angle.minus(currentAngle).getCos();
        if (driveFeedforwardEnabled && driveFeedforward != null) {
            double driveVolts = calculateFeedforwardVolts(optimizedState.speedMetersPerSecond);
            setDriveMotorVoltage(driveVolts);
        } else {
            setDriveMotor(optimizedState.speedMetersPerSecond / config.maxSpeedMetersPerSecond());
        }
        setRotationMotor(
                rotationPidController.calculate(
                        MathUtil.angleModulus(currentAngle.getRadians()),
                        optimizedState.angle.getRadians()));
    }

    public void stop() {
        driveMotor.stopMotor();
        rotationMotor.stopMotor();
        lastDriveCommand = 0;
        lastSteerCommand = 0;
        lastSetpointSpeedMetersPerSecond = 0.0;
        hasPreviousSetpoint = false;
    }

    public void setToAngle(double angle) {
        setDesiredState(new SwerveModuleState(0, new Rotation2d(angle)));
    }

    public void setNeutralMode(boolean brake){
        setNeutralMode(MotorNeutralMode.fromBoolean(brake));
    }

    public void setNeutralMode(MotorNeutralMode mode){
        driveMotor.setNeutralMode(mode);
        rotationMotor.setNeutralMode(mode);
    }

    public void refresh() {
        if (simulation != null) {
            simulation.applyToSensors();
        } else {
            if (encoder != null) {
                encoder.update();
            }
            if (driveEncoder != null && driveEncoder != encoder) {
                driveEncoder.update();
            }
            if (steerEncoder != null && steerEncoder != encoder && steerEncoder != driveEncoder) {
                steerEncoder.update();
            }
        }
    }

    public void attachSimulation(ca.frc6390.athena.drivetrains.swerve.sim.SwerveModuleSimulation simulation) {
        this.simulation = simulation;
    }

    private double getVoltageLimit() {
        if (RobotBase.isSimulation()) {
            return nominalVoltage;
        }
        double battery = RobotController.getBatteryVoltage();
        if (Double.isFinite(battery) && battery > 1e-3) {
            return battery;
        }
        return nominalVoltage;
    }

    private double calculateFeedforwardVolts(double targetSpeedMetersPerSecond) {
        double currentSpeed = hasPreviousSetpoint
                ? lastSetpointSpeedMetersPerSecond
                : targetSpeedMetersPerSecond;
        hasPreviousSetpoint = true;
        lastSetpointSpeedMetersPerSecond = targetSpeedMetersPerSecond;
        return driveFeedforward.calculateWithVelocities(currentSpeed, targetSpeedMetersPerSecond);
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        node.putDouble("driveMotorPosition", driveEncoder != null ? driveEncoder.getPosition() : 0.0);
        encoder.networkTables(node.child("Encoder"));

        if (nt.enabled(RobotNetworkTables.Flag.SWERVE_MODULE_DEBUG)) {
            driveMotor.networkTables(node.child("DriveMotor"));
            rotationMotor.networkTables(node.child("SteerMotor"));
            node.putDouble("offset", encoder != null ? encoder.getOffset() : 0.0);
            node.putDouble("startupOffset", startUpOffset);
        }

        return node;
    }
}
