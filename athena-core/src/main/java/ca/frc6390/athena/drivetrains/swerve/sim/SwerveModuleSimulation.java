package ca.frc6390.athena.drivetrains.swerve.sim;

import ca.frc6390.athena.drivetrains.swerve.SwerveModule;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleSimConfig;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.sim.MotorSimType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleSimulation {

    private final SwerveModule module;
    private final SwerveModuleConfig config;
    private final SwerveModuleSimConfig simConfig;
    private final SwerveSimulationConfig drivetrainConfig;
    private final FlywheelSim driveSim;
    private final FlywheelSim steerSim;
    private final double wheelRadius;
    private final double wheelCircumference;
    private final double moduleCount;
    private final double driveOutputPerMotorRotation;
    private final double rotationMotorOutputPerMotorRotation;
    private final double encoderOutputPerRotation;
    private final Encoder driveEncoder;
    private final Encoder steerEncoder;
    private final Encoder steerMotorEncoder;
    private final double nominalVoltage;

    private double wheelPositionMeters = 0;
    private double wheelVelocityMetersPerSecond = 0;
    private double steerAngleRadians = 0;
    private double steerAngularVelocity = 0;
    private double lastDriveCurrent = 0;
    private double lastSteerCurrent = 0;

    public SwerveModuleSimulation(SwerveModule module, SwerveSimulationConfig drivetrainConfig, int moduleCount) {
        this.module = module;
        this.config = module.getConfigData();
        this.simConfig = config.sim() != null ? config.sim() : SwerveModuleSimConfig.fromMotors(null, null);
        this.drivetrainConfig = drivetrainConfig;
        this.moduleCount = Math.max(1, moduleCount);
        this.wheelRadius = config.wheelDiameter() / 2.0;
        this.wheelCircumference = config.wheelDiameter() * Math.PI;
        this.driveEncoder = module.getDriveMotorController() != null ? module.getDriveMotorController().getEncoder() : null;
        this.steerMotorEncoder = module.getSteerMotorController() != null ? module.getSteerMotorController().getEncoder() : null;
        this.steerEncoder = module.getSteerEncoder();
        this.nominalVoltage = drivetrainConfig.getNominalVoltage();

        this.driveOutputPerMotorRotation = config.driveMotor() != null && config.driveMotor().encoderConfig != null
                ? config.driveMotor().encoderConfig.gearRatio
                : 1.0;
        this.rotationMotorOutputPerMotorRotation = config.rotationMotor() != null && config.rotationMotor().encoderConfig != null
                ? config.rotationMotor().encoderConfig.gearRatio
                : 1.0;
        this.encoderOutputPerRotation = config.encoder() != null ? config.encoder().gearRatio : 1.0;

        DCMotor driveMotor = resolveMotor(simConfig.driveMotorType(), config.driveMotor());
        DCMotor steerMotor = resolveMotor(simConfig.steerMotorType(), config.rotationMotor());

        double driveReduction = Math.abs(driveOutputPerMotorRotation) > 1e-6
                ? 1.0 / driveOutputPerMotorRotation
                : 1.0;

        double steerReduction = Math.abs(rotationMotorOutputPerMotorRotation) > 1.0
                ? rotationMotorOutputPerMotorRotation
                : (Math.abs(rotationMotorOutputPerMotorRotation) > 1e-6
                        ? 1.0 / rotationMotorOutputPerMotorRotation
                        : 1.0);

        double driveMoi = simConfig.driveMomentOfInertia() > 0
                ? simConfig.driveMomentOfInertia()
                : computeDriveMomentOfInertia(drivetrainConfig.getRobotMassKg(), wheelRadius, this.moduleCount);

        double steerMoi = simConfig.steerMomentOfInertia() > 0 ? simConfig.steerMomentOfInertia() : 0.01;

        driveSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(driveMotor, driveMoi, driveReduction), driveMotor);
        steerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(steerMotor, steerMoi, steerReduction), steerMotor);
    }

    private static double computeDriveMomentOfInertia(double robotMassKg, double wheelRadius, double moduleCount) {
        if (robotMassKg <= 0) {
            return 0.02;
        }
        return (robotMassKg * wheelRadius * wheelRadius) / moduleCount;
    }

    private static DCMotor resolveMotor(MotorSimType motor, MotorControllerConfig config) {
        if (motor != null) {
            return motor.createSimMotor(1);
        }

        // Fallback to a generic motor model if type is unknown.
        return DCMotor.getFalcon500(1);
    }

    public void update(double dtSeconds) {
        double drivePercent = MathUtil.clamp(module.getDriveCommandPercent(), -1.0, 1.0);
        double steerPercent = MathUtil.clamp(module.getSteerCommandPercent(), -1.0, 1.0);

        driveSim.setInputVoltage(drivePercent * nominalVoltage);
        steerSim.setInputVoltage(steerPercent * nominalVoltage);

        driveSim.update(dtSeconds);
        steerSim.update(dtSeconds);

        double wheelAngularVelocity = driveSim.getAngularVelocityRadPerSec();
        wheelVelocityMetersPerSecond = wheelAngularVelocity * wheelRadius;
        wheelPositionMeters += wheelVelocityMetersPerSecond * dtSeconds;

        steerAngularVelocity = steerSim.getAngularVelocityRadPerSec();
        steerAngleRadians = MathUtil.angleModulus(steerAngleRadians + steerAngularVelocity * dtSeconds);

        lastDriveCurrent = driveSim.getCurrentDrawAmps();
        lastSteerCurrent = steerSim.getCurrentDrawAmps();

        applyToSensors();
    }

    public void applyToSensors() {
        if (driveEncoder != null) {
            double wheelRotations = wheelCircumference > 1e-6 ? wheelPositionMeters / wheelCircumference : 0;
            double wheelVelocityRotationsPerSecond = wheelCircumference > 1e-6
                    ? wheelVelocityMetersPerSecond / wheelCircumference
                    : 0;

            // Simulate encoder in wheel rotations so conversion factors (usually set to wheel circumference)
            // produce correct linear distances.
            driveEncoder.setSimulatedState(wheelRotations, wheelVelocityRotationsPerSecond);
        }

        if (steerEncoder != null) {
            double moduleRotations = steerAngleRadians / (2.0 * Math.PI);
            double encoderRotations = Math.abs(encoderOutputPerRotation) > 1e-6
                    ? moduleRotations / encoderOutputPerRotation
                    : moduleRotations;
            double encoderVelocity = steerAngularVelocity / (2.0 * Math.PI);
            encoderVelocity = Math.abs(encoderOutputPerRotation) > 1e-6
                    ? encoderVelocity / encoderOutputPerRotation
                    : encoderVelocity;

            steerEncoder.setSimulatedState(encoderRotations, encoderVelocity);
        }

        if (steerMotorEncoder != null) {
            double moduleRotations = steerAngleRadians / (2.0 * Math.PI);
            double motorRotations = Math.abs(rotationMotorOutputPerMotorRotation) > 1e-6
                    ? moduleRotations / rotationMotorOutputPerMotorRotation
                    : moduleRotations;
            double motorVelocity = steerAngularVelocity / (2.0 * Math.PI);
            motorVelocity = Math.abs(rotationMotorOutputPerMotorRotation) > 1e-6
                    ? motorVelocity / rotationMotorOutputPerMotorRotation
                    : motorVelocity;
            steerMotorEncoder.setSimulatedState(motorRotations, motorVelocity);
        }
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(wheelVelocityMetersPerSecond, new Rotation2d(steerAngleRadians));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(wheelPositionMeters, new Rotation2d(steerAngleRadians));
    }

    public double getDriveCurrentAmps() {
        return lastDriveCurrent;
    }

    public double getSteerCurrentAmps() {
        return lastSteerCurrent;
    }

    public double getWheelPositionMeters() {
        return wheelPositionMeters;
    }

    public double getSteerAngleRadians() {
        return steerAngleRadians;
    }

    public void reset(double drivePositionMeters, double steerAngleRadians) {
        this.wheelPositionMeters = drivePositionMeters;
        this.wheelVelocityMetersPerSecond = 0;
        this.steerAngleRadians = MathUtil.angleModulus(steerAngleRadians);
        this.steerAngularVelocity = 0;
        applyToSensors();
    }
}
