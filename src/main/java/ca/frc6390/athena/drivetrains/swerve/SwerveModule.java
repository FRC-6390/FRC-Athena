package ca.frc6390.athena.drivetrains.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class SwerveModule {

    private final SwerveModuleConfig config;

    private final TalonFX driveMotor;
    private final TalonFX rotationMotor;
    private CANcoder encoder;
    private final PIDController rotationPidController;

    private StatusSignal<AngularVelocity> driveVel;
    private StatusSignal<Angle> encoderPos, drivePos, rotationPos;

    // SWERVE MOTOR RECORD
    public record SwerveMotor(int motor, boolean motorReversed, double gearRatio, double maxSpeedMetersPerSecond,
            String canbus) {
        public SwerveMotor(int motor, boolean motorReversed, double maxSpeedMetersPerSecond, double gearRatio) {
            this(motor, motorReversed, gearRatio, maxSpeedMetersPerSecond, "can");
        }

        public SwerveMotor(int motor, boolean motorReversed, double maxSpeedMetersPerSecond) {
            this(motor, motorReversed, 1, maxSpeedMetersPerSecond, "can");
        }

        public SwerveMotor(int motor, boolean motorReversed, double maxSpeedMetersPerSecond, String canbus) {
            this(motor, motorReversed, 1, maxSpeedMetersPerSecond, canbus);
        }

        public SwerveMotor(int motor, boolean motorReversed) {
            this(motor, motorReversed, 1, 0, "can");
        }

        public SwerveMotor(int motor, boolean motorReversed, String canbus) {
            this(motor, motorReversed, 1, 0, canbus);
        }

        public TalonFXConfiguration generateTalonFXConfig(double currentLimit) {
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            CurrentLimitsConfigs driveCurrentLimit = new CurrentLimitsConfigs();
            driveCurrentLimit.SupplyCurrentLimitEnable = true;
            driveCurrentLimit.SupplyCurrentLimit = currentLimit;
            driveConfig.withCurrentLimits(driveCurrentLimit);

            InvertedValue isInverted = motorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
            driveConfig.MotorOutput.withInverted(isInverted);

            return driveConfig;
        } 
    }

    public record SwerveModuleConfig(Translation2d module_location, double wheelDiameter, SwerveMotor driveMotor, SwerveMotor rotationMotor, PIDController rotationPID, double offsetRadians, int encoder, double encoderGearRatio) {
        public SwerveModuleConfig(Translation2d module_location, double wheelDiameter, SwerveMotor driveMotor, SwerveMotor rotationMotor, PIDController rotationPID, double offsetRadian) {
            this(module_location, wheelDiameter, driveMotor, rotationMotor, rotationPID, offsetRadian, -1, -1);
        }
    }

    public SwerveModule(SwerveModuleConfig config) {
        this.config = config;
        // MODULE ROTATION PID
        rotationPidController = config.rotationPID();

        // ROTATION PID ENABLING -180 TO 180
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

        // MOTOR INITIALIZATION
        driveMotor = new TalonFX(config.driveMotor().motor(), config.driveMotor().canbus());
        rotationMotor = new TalonFX(config.rotationMotor().motor(), config.rotationMotor().canbus());

        //CONFIGS
        rotationMotor.getConfigurator().apply(config.rotationMotor().generateTalonFXConfig(60));
        driveMotor.getConfigurator().apply(config.driveMotor().generateTalonFXConfig(60));

        if (config.encoder() > -1) {
            encoder = new CANcoder(config.encoder(), config.rotationMotor().canbus());
            encoderPos = encoder.getPosition();

            CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
            encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; //180 <-> (-180)
            encoderConfig.MagnetSensor.MagnetOffset = config.offsetRadians();
            encoder.getConfigurator().apply(encoderConfig);
        }

        // DRIVE MOTOR POSITION AND VELOCITY
        drivePos = driveMotor.getRotorPosition();
        driveVel = driveMotor.getRotorVelocity();
        rotationPos = rotationMotor.getRotorPosition();

        // RESET ENCODERS AND BRAKE MODE MODULES
        resetEncoders();
        lock();
    }

    public double getDriveMotorVelocity() {

        return driveVel.getValueAsDouble() * config.driveMotor().gearRatio() * Math.PI * config.wheelDiameter();

    }

    public Translation2d getModuleLocation() {
        return config.module_location();
    }

    public double getDriveMotorPosition() {

        return drivePos.getValueAsDouble() * config.driveMotor().gearRatio() * Math.PI * config.wheelDiameter();
    }

    public double getModuleRotationRadians() {
        if (encoder != null) {
            return getEncoderRadians();
        }

        return getRotationMotorRadians();
    }

    public double getModuleRotationRadiansAbsolute(){

        if(encoder != null) {
            return getEncoderRadiansAbsolute();
        }

        return getEncoderRadiansAbsolute();
    }

    public double getEncoderRadians() {
        return (encoderPos.getValueAsDouble() * 360 * Math.PI / 180d);
    }

    public double getEncoderRadiansAbsolute() {
        return getEncoderRadians() * config.encoderGearRatio();
    }

    public double getOffsetRadians() {
        return config.offsetRadians();
    }

    public double getRotationMotorRadiansAbsolute() {
        return getRotationMotorRadians() * config.rotationMotor().gearRatio();
    }
  

    public double getRotationMotorRadians() {
        return (rotationPos.getValueAsDouble() * 360 * Math.PI / 180d);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        rotationMotor.setPosition(encoderPos.getValueAsDouble());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getModuleRotationRadians()));
    }

    public SwerveModulePosition getPostion() {
        return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getModuleRotationRadians()));
    }

    public void setDriveMotor(double speed) {
        driveMotor.set(speed);
    }

    public void setRotationMotor(double speed) {
        rotationMotor.set(speed);
    }

    public void setDesiredState(SwerveModuleState state) {
        refresh();

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();

            return;
        }

        state.optimize(getState().angle);
        driveMotor.set(state.speedMetersPerSecond / config.driveMotor().maxSpeedMetersPerSecond());

        rotationMotor.set(rotationPidController.calculate(-getEncoderRadians(), -state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void setToAngle(double angle) {
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(angle));

        rotationMotor.set(rotationPidController.calculate(-getEncoderRadians(), -state.angle.getRadians()));
    }

    public void lock() {
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void unlock() {
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        rotationMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void refresh() {
        drivePos.refresh();
        driveVel.refresh();
        encoderPos.refresh();
        rotationPos.refresh();
    }

}