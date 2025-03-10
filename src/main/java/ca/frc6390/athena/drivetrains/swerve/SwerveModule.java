package ca.frc6390.athena.drivetrains.swerve;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.EncoderConfig;
import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import ca.frc6390.athena.devices.MotorController;
import ca.frc6390.athena.devices.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
public class SwerveModule implements RobotSendableDevice {
    
    private final SwerveModuleConfig config;
    private final MotorController driveMotor;
    private final MotorController rotationMotor;
    private final Encoder encoder;
    private final PIDController rotationPidController;
    // SWERVE MOTOR RECORD
    public record SwerveModuleConfig(Translation2d module_location, double wheelDiameter, double maxSpeedMetersPerSecond, MotorControllerConfig driveMotor, MotorControllerConfig rotationMotor, PIDController rotationPID, EncoderConfig encoder) {
        public SwerveModuleConfig(Translation2d module_location, double wheelDiameter, double maxSpeedMetersPerSecond, MotorControllerConfig driveMotor, MotorControllerConfig rotationMotor, PIDController rotationPID) {
            this(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, rotationMotor.encoderConfig);
        }

        public SwerveModuleConfig setCanbus(String canbus){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setCanbus(canbus), rotationMotor.setCanbus(canbus), rotationPID, encoder.setCanbus(canbus));
        }

        public SwerveModuleConfig setP(double kP){
            return setPID(new PIDController(kP,0,0));
        }

        public SwerveModuleConfig setPID(double kP, double kI, double kD){
            return setPID(new PIDController(kP, kI, kD));
        }

        public SwerveModuleConfig setPID(PIDController rotationPID){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder);
        }

        public SwerveModuleConfig setEncoder(EncoderConfig encoder){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder);
        }

        public SwerveModuleConfig setEncoder(EncoderType encoder){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder().setEncoderType(encoder));
        }

        public SwerveModuleConfig setOffset(double offset){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder.setOffset(offset));
        }

        public SwerveModuleConfig setLocation(Translation2d module_location){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder);
        }

        public SwerveModuleConfig setDriveID(int id){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setID(id), rotationMotor, rotationPID, encoder); 
        }

        public SwerveModuleConfig setSteerID(int id){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor.setID(id), rotationPID, encoder); 
        }

        public SwerveModuleConfig setEncoderID(int id){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder.setID(id)); 
        }

        public SwerveModuleConfig setDriveInverted(boolean inverted){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setInverted(inverted), rotationMotor, rotationPID, encoder); 
        }

        public SwerveModuleConfig setSteerInverted(boolean inverted){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor.setInverted(inverted), rotationPID, encoder); 
        }

        public SwerveModuleConfig setEncoderInverted(boolean inverted){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder.setInverted(inverted)); 
        }

        public SwerveModuleConfig setCurrentLimit(double currentLimit) {
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setCurrentLimit(currentLimit), rotationMotor.setCurrentLimit(currentLimit),rotationPID, encoder); 
        }

        public static Translation2d[] generateModuleLocations(double trackwidth, double wheelbase) {
            Translation2d FRONT_LEFT = new Translation2d(trackwidth/2, wheelbase/2);
            Translation2d FRONT_RIGHT = new Translation2d(trackwidth/2, -wheelbase/2);
            Translation2d BACK_LEFT = new Translation2d(-trackwidth/2, wheelbase/2);
            Translation2d BACK_RIGHT = new Translation2d(-trackwidth/2, -wheelbase/2);
            return new Translation2d[]{FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
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
        driveMotor.setSpeed(speed);
    }

    public void setRotationMotor(double speed) {
        rotationMotor.setSpeed(speed);
    }

    public void setDesiredState(SwerveModuleState state) {
        refresh();

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();

            return;
        }

        state.optimize(getState().angle);
        // state.speedMetersPerSecond *= state.angle.minus(encoder.getRotation2d()).getCos();
        setDriveMotor(state.speedMetersPerSecond / config.maxSpeedMetersPerSecond());
        setRotationMotor(rotationPidController.calculate(MathUtil.angleModulus(getEncoderPosition().getRadians()), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.stopMotor();
        rotationMotor.stopMotor();
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
        encoder.update();
        driveMotor.getEncoder().update();
        // System.out.println(driveMotor.getEncoder().getGearRatio());
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 2));
        layout.addDouble("Encoder Rotations", () -> getEncoderPosition().getRotations()).withSize(1, 1).withPosition(1, 1);
        layout.addDouble("Drive Motor Position", () -> getDriveMotorPosition()).withSize(1, 1).withPosition(1, 2);
        return layout;
    }
}