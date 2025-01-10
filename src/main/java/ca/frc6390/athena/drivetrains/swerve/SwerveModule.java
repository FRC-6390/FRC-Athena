package ca.frc6390.athena.drivetrains.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.Map;

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

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
public class SwerveModule {

    private final SwerveModuleConfig config;

    private final TalonFX driveMotor;
    private final TalonFX rotationMotor;
    private NeutralModeValue mode;
    private CANcoder encoder;
    private final PIDController rotationPidController;
    
    private StatusSignal<AngularVelocity> driveVel;
    private StatusSignal<Angle> encoderPos, drivePos, rotationPos;

    // SWERVE MOTOR RECORD
    public record SwerveMotor(int id, boolean motorReversed, double gearRatio, double maxSpeedMetersPerSecond,
            String canbus) {
        public SwerveMotor(int id, boolean motorReversed, double maxSpeedMetersPerSecond, double gearRatio) {
            this(id, motorReversed, gearRatio, maxSpeedMetersPerSecond, "can");
        }

        public SwerveMotor(int id, boolean motorReversed, double maxSpeedMetersPerSecond) {
            this(id, motorReversed, 1, maxSpeedMetersPerSecond, "can");
        }

        public SwerveMotor(int id, boolean motorReversed, double maxSpeedMetersPerSecond, String canbus) {
            this(id, motorReversed, 1, maxSpeedMetersPerSecond, canbus);
        }

        public SwerveMotor(int id, boolean motorReversed) {
            this(id, motorReversed, 1, 0, "can");
        }

        public SwerveMotor(int id, boolean motorReversed, String canbus) {
            this(id, motorReversed, 1, 0, canbus);
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

        public SwerveMotor reversed(){
            return new SwerveMotor(id, true, gearRatio, maxSpeedMetersPerSecond, canbus);
        }

        public SwerveEncoder asEncoder(double offsetRadian) {
            return new SwerveEncoder(id, offsetRadian, gearRatio, canbus);
        }
    }

    public record SwerveEncoder(int id, double offsetRotations, double gearRatio, String canbus) {

        public SwerveEncoder(int id, double offsetRotations, double gearRatio) {
            this(id, offsetRotations, gearRatio, "can");
        }

        public SwerveEncoder(int id, double offsetRotations) {
            this(id, offsetRotations, 1, "can");
        }

        public SwerveEncoder(int id, double offsetRotations, String canbus) {
            this(id, offsetRotations, 1, canbus);
        }

        public CANcoderConfiguration generateConfig() {
           return generateConfig(offsetRotations);
        } 

        public CANcoderConfiguration generateConfig(double offset) {
            CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
            encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; //180 <-> (-180)
            encoderConfig.MagnetSensor.MagnetOffset = offset;
           return encoderConfig;
        } 
    }

    public record SwerveModuleConfig(Translation2d module_location, double wheelDiameter, SwerveMotor driveMotor, SwerveMotor rotationMotor, PIDController rotationPID, SwerveEncoder encoder) {
        public SwerveModuleConfig(Translation2d module_location, double wheelDiameter, SwerveMotor driveMotor, SwerveMotor rotationMotor, PIDController rotationPID, double offsetRotations) {
            this(module_location, wheelDiameter, driveMotor, rotationMotor, rotationPID, rotationMotor.asEncoder(offsetRotations));
        }
    }

    public SwerveModule(SwerveModuleConfig config) {
        this.config = config;
        // MODULE ROTATION PID
        rotationPidController = config.rotationPID();

        // ROTATION PID ENABLING -180 TO 180
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

        // MOTOR INITIALIZATION
        driveMotor = new TalonFX(config.driveMotor().id(), config.driveMotor().canbus());
        rotationMotor = new TalonFX(config.rotationMotor().id(), config.rotationMotor().canbus());

        //CONFIGS
        rotationMotor.getConfigurator().apply(config.rotationMotor().generateTalonFXConfig(60));
        driveMotor.getConfigurator().apply(config.driveMotor().generateTalonFXConfig(60));

        if (config.encoder().id() != config.rotationMotor().id()) {
            encoder = new CANcoder(config.encoder().id(), config.encoder().canbus());
            encoderPos = encoder.getAbsolutePosition();
            encoder.getConfigurator().apply(config.encoder().generateConfig());
        }else {
            encoderPos = rotationMotor.getRotorPosition();
        }

        // DRIVE MOTOR POSITION AND VELOCITY
        drivePos = driveMotor.getRotorPosition();
        driveVel = driveMotor.getRotorVelocity();
        rotationPos = rotationMotor.getRotorPosition();
        // RESET ENCODERS AND BRAKE MODE MODULES
        resetEncoders();
        setNeutralMode(NeutralModeValue.Brake);
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

    public double getEncoderRadians() {
        return getEncoderDegrees() * Math.PI / 180d;
    }

    public double getEncoderRotations() {
        return Math.IEEEremainder(encoderPos.getValueAsDouble() * config.encoder().gearRatio(), 1);
    }

    public double getEncoderDegrees() {
        return Math.IEEEremainder(encoderPos.getValueAsDouble() * config.encoder().gearRatio() * 360, 360);
    }

    public double getOffsetRotations() {
        return config.encoder().offsetRotations();
    }

    public void setOffsetRotations(double offset) {
        encoder.getConfigurator().apply(config.encoder().generateConfig(offset));
    }

    public double getOffsetDegrees() {
        return (config.encoder().offsetRotations() * 360);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        rotationMotor.setPosition(encoderPos.getValueAsDouble());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getEncoderRadians()));
    }

    public SwerveModulePosition getPostion() {
        return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getEncoderRadians()));
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

        rotationMotor.set(rotationPidController.calculate(getEncoderRadians(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void setToAngle(double angle) {
        setDesiredState(new SwerveModuleState(0, new Rotation2d(angle)));
    }

    public void setNeutralMode(boolean brake){
        setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setNeutralMode(NeutralModeValue mode){
        this.mode = mode;
        driveMotor.setNeutralMode(mode);
        rotationMotor.setNeutralMode(mode);
    }

    public NeutralModeValue getNeutralMode(){
        return mode;
    }

    public void refresh() {
        drivePos.refresh();
        driveVel.refresh();
        encoderPos.refresh();
        rotationPos.refresh();
    }

    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        layout.addBoolean("Locked", () -> mode == NeutralModeValue.Brake ? true : false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 1);
        layout.addDouble("Offset Rotations", this::getOffsetRotations).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 2);
        layout.addDouble("Encoder Rotations", this::getEncoderRotations).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 3);
        return layout;
    }
}