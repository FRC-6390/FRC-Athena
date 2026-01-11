package ca.frc6390.athena.drivetrains.swerve;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.EncoderConfig;
import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import ca.frc6390.athena.devices.MotorController;
import ca.frc6390.athena.devices.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import ca.frc6390.athena.sim.MotorSimType;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;
public class SwerveModule implements RobotSendableDevice {
    
    private final SwerveModuleConfig config;
    private final MotorController driveMotor;
    private final MotorController rotationMotor;
    private final Encoder encoder;
    private final PIDController rotationPidController;
    private final double startUpOffset;
    private double lastDriveCommand = 0;
    private double lastSteerCommand = 0;
    private ca.frc6390.athena.drivetrains.swerve.sim.SwerveModuleSimulation simulation;
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
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder().setEncoderType(encoder), sim);
        }

        public SwerveModuleConfig setOffset(double offset){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder.setOffset(offset), sim);
        }

        public SwerveModuleConfig setLocation(Translation2d module_location){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder, sim);
        }

        public SwerveModuleConfig setDriveID(int id){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setID(id), rotationMotor, rotationPID, encoder, sim); 
        }

        public SwerveModuleConfig setSteerID(int id){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor.setID(id), rotationPID, encoder, sim); 
        }

        public SwerveModuleConfig setEncoderID(int id){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder.setID(id), sim); 
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
        driveMotor = MotorController.fromConfig(config.driveMotor);

        rotationMotor = MotorController.fromConfig(config.rotationMotor);

        encoder = Encoder.fromConfig(config.encoder);

        resetEncoders();
        setNeutralMode(MotorNeutralMode.Brake);
        this.startUpOffset = encoder.getOffset();
    }

    public double getDriveMotorVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public Translation2d getModuleLocation() {
        return config.module_location();
    }

    public double getDriveMotorPosition() {
        return driveMotor.getEncoder().getPosition();
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
        driveMotor.getEncoder().setPosition(0);
        rotationMotor.getEncoder().setPosition(encoder.getAbsolutePosition());
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

    public void setRotationMotor(double speed) {
        lastSteerCommand = speed;
        rotationMotor.setSpeed(speed);
    }

    public double getDriveCommandPercent() {
        return lastDriveCommand;
    }

    public double getSteerCommandPercent() {
        return lastSteerCommand;
    }

    public void setDesiredState(SwerveModuleState state) {
        refresh();

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();

            return;
        }

        state.optimize(getState().angle);
        state.speedMetersPerSecond *= state.angle.minus(encoder.getRotation2d()).getCos();
        setDriveMotor(state.speedMetersPerSecond / config.maxSpeedMetersPerSecond());
        setRotationMotor(rotationPidController.calculate(MathUtil.angleModulus(getEncoderPosition().getRadians()), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.stopMotor();
        rotationMotor.stopMotor();
        lastDriveCommand = 0;
        lastSteerCommand = 0;
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

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 2));
        layout.addDouble("Drive Motor Position", () -> getDriveMotorPosition()).withSize(1, 1).withPosition(1, 2);
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
}
