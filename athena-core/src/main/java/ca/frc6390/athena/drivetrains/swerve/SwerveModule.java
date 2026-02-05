package ca.frc6390.athena.drivetrains.swerve;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.sim.MotorSimType;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private double lastSetpointTimestampSeconds = Double.NaN;
    private ca.frc6390.athena.drivetrains.swerve.sim.SwerveModuleSimulation simulation;
    private double shuffleboardPeriodSeconds = ca.frc6390.athena.core.RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();
    // SWERVE MOTOR RECORD
    public record SwerveModuleConfig(Translation2d module_location, double wheelDiameter, double maxSpeedMetersPerSecond, MotorControllerConfig driveMotor, MotorControllerConfig rotationMotor, PIDController rotationPID, EncoderConfig encoder, SwerveModuleSimConfig sim) {
        public SwerveModuleConfig(Translation2d module_location, double wheelDiameter, double maxSpeedMetersPerSecond, MotorControllerConfig driveMotor, MotorControllerConfig rotationMotor, PIDController rotationPID, EncoderConfig encoder) {
            this(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder, SwerveModuleSimConfig.fromMotors(null, null));
        }
        public SwerveModuleConfig(Translation2d module_location, double wheelDiameter, double maxSpeedMetersPerSecond, MotorControllerConfig driveMotor, MotorControllerConfig rotationMotor, PIDController rotationPID) {
            this(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, rotationMotor.encoderConfig, SwerveModuleSimConfig.fromMotors(null, null));
        }

        //cloning
        public SwerveModuleConfig(SwerveModuleConfig other) {
            this(
                new Translation2d(other.module_location().getX(), other.module_location().getY()),
                other.wheelDiameter(),
                other.maxSpeedMetersPerSecond(),
                other.driveMotor(),
                other.rotationMotor(),
                new PIDController(other.rotationPID().getP(), other.rotationPID().getI(), other.rotationPID().getD()),
                other.encoder(),
                other.sim()
            );
        }

        public SwerveModuleConfig setCanbus(String canbus){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setCanbus(canbus), rotationMotor.setCanbus(canbus), rotationPID, encoder.setCanbus(canbus), sim);
        }

        public SwerveModuleConfig setP(double kP){
            return setPID(new PIDController(kP,0,0));
        }

        public SwerveModuleConfig setPID(double kP, double kI, double kD){
            return setPID(new PIDController(kP, kI, kD));
        }

        public SwerveModuleConfig setPID(PIDController rotationPID){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder, sim);
        }

        public SwerveModuleConfig setEncoder(EncoderConfig encoder){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder, sim);
        }

        public SwerveModuleConfig setEncoder(EncoderType encoder){
            EncoderConfig cfg = encoder() != null ? encoder().setType(encoder) : EncoderConfig.type(encoder, 0);
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, cfg, sim);
        }

        public SwerveModuleConfig setOffset(double offset){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder.setOffset(offset), sim);
        }

        public SwerveModuleConfig setLocation(Translation2d module_location){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder, sim);
        }

        public SwerveModuleConfig setDriveID(int id){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setId(id), rotationMotor, rotationPID, encoder, sim); 
        }

        public SwerveModuleConfig setSteerID(int id){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor.setId(id), rotationPID, encoder, sim); 
        }

        public SwerveModuleConfig setEncoderID(int id){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder.setId(id), sim); 
        }

        public SwerveModuleConfig setDriveInverted(boolean inverted){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setInverted(inverted), rotationMotor, rotationPID, encoder, sim); 
        }

        public SwerveModuleConfig setSteerInverted(boolean inverted){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor.setInverted(inverted), rotationPID, encoder, sim); 
        }

        public SwerveModuleConfig setEncoderInverted(boolean inverted){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder.setInverted(inverted), sim); 
        }

        public SwerveModuleConfig setCurrentLimit(double currentLimit) {
            return setDriveCurrentLimit(currentLimit).setSteerCurrentLimit(currentLimit);
        }

        public SwerveModuleConfig setDriveCurrentLimit(double currentLimit) {
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setCurrentLimit(currentLimit), rotationMotor, rotationPID, encoder, sim);
        }

        public SwerveModuleConfig setSteerCurrentLimit(double currentLimit) {
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor.setCurrentLimit(currentLimit), rotationPID, encoder, sim);
        }

        public static Translation2d[] generateModuleLocations(double trackwidth, double wheelbase) {
            Translation2d FRONT_LEFT = new Translation2d(trackwidth/2, wheelbase/2);
            Translation2d FRONT_RIGHT = new Translation2d(trackwidth/2, -wheelbase/2);
            Translation2d BACK_LEFT = new Translation2d(-trackwidth/2, wheelbase/2);
            Translation2d BACK_RIGHT = new Translation2d(-trackwidth/2, -wheelbase/2);
            return new Translation2d[]{FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
        }

        public SwerveModuleConfig setSim(SwerveModuleSimConfig sim) {
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder, sim);
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
        driveMotor = HardwareFactories.motor(config.driveMotor);

        rotationMotor = HardwareFactories.motor(config.rotationMotor);

        encoder = HardwareFactories.encoder(config.encoder);
        driveEncoder = driveMotor.getEncoder();
        steerEncoder = rotationMotor.getEncoder();

        resetEncoders();
        setNeutralMode(MotorNeutralMode.Brake);
        this.startUpOffset = encoder.getOffset();
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
        return encoder.getAbsoluteRotation2d();
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
            steerEncoder.setPosition(encoder.getAbsolutePosition());
        } else {
            DriverStation.reportWarning("Steer encoder not configured for swerve module; position reset skipped.", false);
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMotorVelocity(), getEncoderPosition());
    }

    public SwerveModulePosition getPostion() {
        return new SwerveModulePosition(getDriveMotorPosition(), getEncoderPosition());
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

        state.optimize(getState().angle);
        state.speedMetersPerSecond *= state.angle.minus(encoder.getRotation2d()).getCos();
        if (driveFeedforwardEnabled && driveFeedforward != null) {
            double driveVolts = calculateFeedforwardVolts(state.speedMetersPerSecond);
            setDriveMotorVoltage(driveVolts);
        } else {
            setDriveMotor(state.speedMetersPerSecond / config.maxSpeedMetersPerSecond());
        }
        setRotationMotor(rotationPidController.calculate(MathUtil.angleModulus(getEncoderPosition().getRadians()), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.stopMotor();
        rotationMotor.stopMotor();
        lastDriveCommand = 0;
        lastSteerCommand = 0;
        lastSetpointSpeedMetersPerSecond = 0.0;
        lastSetpointTimestampSeconds = Double.NaN;
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
            encoder.update();
            driveMotor.getEncoder().update();
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
        double now = Timer.getFPGATimestamp();
        double currentSpeed = Double.isFinite(lastSetpointTimestampSeconds)
                ? lastSetpointSpeedMetersPerSecond
                : targetSpeedMetersPerSecond;
        lastSetpointTimestampSeconds = now;
        lastSetpointSpeedMetersPerSecond = targetSpeedMetersPerSecond;
        return driveFeedforward.calculateWithVelocities(currentSpeed, targetSpeedMetersPerSecond);
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 2));
        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;
        layout.addDouble(
                "Drive Motor Position",
                ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getDriveMotorPosition, period))
            .withSize(1, 1)
            .withPosition(1, 2);
        encoder.shuffleboard(layout.getLayout("Encoder", BuiltInLayouts.kList));
        if(level.equals(SendableLevel.DEBUG)){
            driveMotor.shuffleboard(layout.getLayout("Drive Motor", BuiltInLayouts.kList));
            rotationMotor.shuffleboard(layout.getLayout("Steer Motor", BuiltInLayouts.kList));
            
            
            ShuffleboardLayout commandsLayout = layout.getLayout("Quick Commands",BuiltInLayouts.kList);
            {
                commandsLayout.add("Set Offset", new InstantCommand(() -> setOffset(encoder.getRawAbsoluteValue()))).withWidget(BuiltInWidgets.kCommand);
                commandsLayout.add("Zero Offset", new InstantCommand(() -> setOffset(0))).withWidget(BuiltInWidgets.kCommand);
                commandsLayout.add("Clear Offset", new InstantCommand(() -> setOffset(startUpOffset))).withWidget(BuiltInWidgets.kCommand);
            }
        }
        

        return layout;
    }

    @Override
    public double getShuffleboardPeriodSeconds() {
        return shuffleboardPeriodSeconds;
    }

    @Override
    public void setShuffleboardPeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds)) {
            return;
        }
        shuffleboardPeriodSeconds = periodSeconds;
    }
}
