package ca.frc6390.athena.drivetrains.swerve;

import com.ctre.phoenix6.hardware.TalonFX;

import ca.frc6390.athena.core.RobotDrivetrain.DriveTrainNeutralMode;

import java.util.Map;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
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

    //should make changes so this can work with any motor as the code is not really motor specific and can fairly easily remove the dependancy on CTRE

    private final SwerveModuleConfig config;

    private final TalonFX driveMotor; //switch to double consumer in future
    private final TalonFX rotationMotor; //switch to double consumer in future
    private DriveTrainNeutralMode mode;
    private final CANcoder encoder; //could possibly be entirely remove and manager by a backend instead
    private final PIDController rotationPidController;
    private double offset; 

    private StatusSignal<AngularVelocity> driveVel; //switch to double suppliers in future
    private StatusSignal<Angle> encoderPos, drivePos; //switch to double suppliers in future

    //private final DoubleConsumer driveMotor, rotationMotor
    //private final DoubleSupplier driveVel, encoderPos;

    // SWERVE MOTOR RECORD
    public record SwerveMotor(int id, double maxSpeedMetersPerSecond,  double gearRatio, String canbus) {
        public SwerveMotor(int id, double maxSpeedMetersPerSecond, double gearRatio) {
            this(id, gearRatio, maxSpeedMetersPerSecond, "can");
        }

        public SwerveMotor(int id, double maxSpeedMetersPerSecond, String canbus) {
            this(id, maxSpeedMetersPerSecond, 1, canbus);
        }

        public SwerveMotor(int id, double maxSpeedMetersPerSecond) {
            this(id, maxSpeedMetersPerSecond, 1,"can");
        }

        public TalonFXConfiguration generateTalonFXConfig(double currentLimit) {
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            CurrentLimitsConfigs driveCurrentLimit = new CurrentLimitsConfigs();
            driveCurrentLimit.SupplyCurrentLimitEnable = true;
            driveCurrentLimit.SupplyCurrentLimit = currentLimit;
            driveConfig.withCurrentLimits(driveCurrentLimit);
            return driveConfig;
        }

        public SwerveEncoder asEncoder(double offsetRadian) {
            return new SwerveEncoder(id, offsetRadian, gearRatio, canbus);
        }
    }

    public record SwerveEncoder(int id, double offsetRadians, double gearRatio, String canbus) {

        public SwerveEncoder(int id, double offsetRadians, double gearRatio) {
            this(id, offsetRadians, gearRatio, "can");
        }

        public SwerveEncoder(int id, double offsetRadians) {
            this(id, offsetRadians, 1, "can");
        }

        public SwerveEncoder(int id, double offsetRadians, String canbus) {
            this(id, offsetRadians, 1, canbus);
        }

        public CANcoderConfiguration generateConfig() {
           return generateConfig(offsetRadians);
        } 

        public CANcoderConfiguration generateConfig(double offset) {
            CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
            encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
           return encoderConfig;
        } 
    }

    public record SwerveModuleConfig(Translation2d module_location, double wheelDiameter, SwerveMotor driveMotor, SwerveMotor rotationMotor, PIDController rotationPID, SwerveEncoder encoder) {
        public SwerveModuleConfig(Translation2d module_location, double wheelDiameter, SwerveMotor driveMotor, SwerveMotor rotationMotor, PIDController rotationPID, double offsetRadians) {
            this(module_location, wheelDiameter, driveMotor, rotationMotor, rotationPID, rotationMotor.asEncoder(offsetRadians));
        }
    }

    public SwerveModule(SwerveModuleConfig config) {
        this.config = config;
        // MODULE ROTATION PID
        rotationPidController = config.rotationPID();

        // ROTATION PID ENABLING -180 TO 180
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

        // MOTOR INITIALIZATION
        driveMotor = new TalonFX(config.driveMotor().id, config.driveMotor().canbus);
        rotationMotor = new TalonFX(config.rotationMotor().id, config.rotationMotor().canbus);

        //CONFIGS
        rotationMotor.getConfigurator().apply(config.rotationMotor().generateTalonFXConfig(60));
        driveMotor.getConfigurator().apply(config.driveMotor().generateTalonFXConfig(60));

        if (config.encoder().id() != config.rotationMotor().id) {
            encoder = new CANcoder(config.encoder().id, config.encoder().canbus);
            encoderPos = encoder.getAbsolutePosition();
            encoder.getConfigurator().apply(config.encoder().generateConfig());
        }else {
            encoderPos = rotationMotor.getRotorPosition();
            encoder = null;
        }

        offset = config.encoder().offsetRadians;

        // DRIVE MOTOR POSITION AND VELOCITY
        drivePos = driveMotor.getRotorPosition();
        driveVel = driveMotor.getRotorVelocity();
        // RESET ENCODERS AND BRAKE MODE MODULES
        resetEncoders();
        setNeutralMode(DriveTrainNeutralMode.Brake);
    }

    public double getDriveMotorVelocity() {
        return driveVel.getValueAsDouble() / config.driveMotor().gearRatio * Math.PI * config.wheelDiameter;
    }

    public Translation2d getModuleLocation() {
        return config.module_location();
    }

    public double getDriveMotorRotations() {
        return drivePos.getValueAsDouble() / config.driveMotor().gearRatio;
    }

    public double getDriveMotorPosition() {
        return getDriveMotorRotations() * Math.PI * config.wheelDiameter;
    }

    public Rotation2d getEncoderPosition() {
        return Rotation2d.fromRotations(encoderPos.getValueAsDouble() * config.encoder().gearRatio).minus(Rotation2d.fromRadians(offset));
    }

    public double getOffset() {
        return offset;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        rotationMotor.setPosition(encoderPos.getValueAsDouble());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMotorVelocity(), getEncoderPosition());
    }

    public SwerveModulePosition getPostion() {
        return new SwerveModulePosition(getDriveMotorPosition(), getEncoderPosition());
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
        rotationMotor.set(-rotationPidController.calculate( MathUtil.angleModulus(getEncoderPosition().getRadians()), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void setToAngle(double angle) {
        setDesiredState(new SwerveModuleState(0, new Rotation2d(angle)));
    }

    public void setNeutralMode(boolean brake){
        setNeutralMode(DriveTrainNeutralMode.fromBoolean(brake));
    }

    public void setNeutralMode(DriveTrainNeutralMode mode){
        this.mode = mode;
        driveMotor.setNeutralMode(mode.asCTRE());
        rotationMotor.setNeutralMode(mode.asCTRE());
    }

    public DriveTrainNeutralMode getNeutralMode(){
        return mode;
    }

    public void refresh() {
        drivePos.refresh();
        driveVel.refresh();
        encoderPos.refresh();
    }

    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        layout.addBoolean("Brake Mode", () -> mode.asBoolean()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 1);
        layout.addDouble("Offset Rotations", this::getOffset).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 2);
        layout.addDouble("Encoder Radians", () -> MathUtil.angleModulus(getEncoderPosition().getRadians())).withSize(1, 1).withPosition(0, 3);
        layout.addDouble("Drive Motor Rotations", () -> getDriveMotorRotations()).withSize(1, 1).withPosition(0, 3);
        layout.add("PID", rotationPidController).withSize(1, 1).withPosition(0, 3);

        return layout;
    }
}