package ca.frc6390.athena.drivetrains.swerve;

import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.Encoder.EncoderConfig;
import ca.frc6390.athena.devices.MotorController;
import ca.frc6390.athena.devices.MotorController.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
public class SwerveModule {
    
    private final SwerveModuleConfig config;
    private final MotorController driveMotor; //switch to double consumer in future
    private final MotorController rotationMotor; //switch to double consumer in future
    private final Encoder encoder; //could possibly be entirely remove and manager by a backend instead
    private final PIDController rotationPidController;
    // SWERVE MOTOR RECORD
    public record SwerveModuleConfig(Translation2d module_location, double wheelDiameter, double maxSpeedMetersPerSecond, MotorControllerConfig driveMotor, MotorControllerConfig rotationMotor, PIDController rotationPID, EncoderConfig encoder) {
        public SwerveModuleConfig(Translation2d module_location, double wheelDiameter, double maxSpeedMetersPerSecond, MotorControllerConfig driveMotor, MotorControllerConfig rotationMotor, PIDController rotationPID) {
            this(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, rotationMotor.encoderConfig());
        }

        public SwerveModuleConfig setCanbus(String canbus){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor.setCanbus(canbus), rotationMotor.setCanbus(canbus), rotationPID, encoder.setCanbus(canbus));
        }

        public SwerveModuleConfig setPID(PIDController rotationPID){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder);
        }

        public SwerveModuleConfig setEncoder(EncoderConfig encoder){
            return new SwerveModuleConfig(module_location, wheelDiameter, maxSpeedMetersPerSecond, driveMotor, rotationMotor, rotationPID, encoder);
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
        return encoder.getRotation2d();
        // return Rotation2d.fromRotations(encoder.getPosition() * config.encoder().gearRatio).minus(Rotation2d.fromRadians(offset));
    }

    public void setOffset(double offset) {
        encoder.setOffset(offset);
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        rotationMotor.getEncoder().setPosition(encoder.getPosition());
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
        setDriveMotor(state.speedMetersPerSecond / config.maxSpeedMetersPerSecond());
        setRotationMotor(-rotationPidController.calculate( MathUtil.angleModulus(getEncoderPosition().getRadians()), state.angle.getRadians()));
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
    }

    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        layout.addDouble("Encoder Radians", () -> MathUtil.angleModulus(getEncoderPosition().getRadians())).withSize(1, 1).withPosition(0, 3);
        layout.addDouble("Drive Motor Position", () -> getDriveMotorPosition()).withSize(1, 1).withPosition(0, 3);
        layout.add("PID", rotationPidController).withSize(1, 1).withPosition(0, 3);

        return layout;
    }
}