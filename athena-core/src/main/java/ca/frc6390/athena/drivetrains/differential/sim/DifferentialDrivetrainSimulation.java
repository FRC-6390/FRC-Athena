package ca.frc6390.athena.drivetrains.differential.sim;

import ca.frc6390.athena.sim.Motor;
import ca.frc6390.athena.sim.MotorSimType;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderGroup;
import ca.frc6390.athena.hardware.imu.Imu;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

/**
 * Lightweight wrapper around WPILib's {@link DifferentialDrivetrainSim} that keeps the drivetrain's
 * encoders and IMU in sync while running inside the simulator.
 */
public class DifferentialDrivetrainSimulation {

    private final DifferentialDrivetrain drivetrain;
    private final DifferentialSimulationConfig config;
    private final DifferentialDrivetrainSim sim;
    private final EncoderGroup leftEncoders;
    private final EncoderGroup rightEncoders;
    private final Imu imu;

    public DifferentialDrivetrainSimulation(DifferentialDrivetrain drivetrain,
                                            DifferentialSimulationConfig config,
                                            EncoderGroup leftEncoders,
                                            EncoderGroup rightEncoders) {
        this.drivetrain = drivetrain;
        this.config = config;
        this.leftEncoders = leftEncoders;
        this.rightEncoders = rightEncoders;
        this.imu = drivetrain.imu().device();

        DCMotor motorModel = resolveMotor(config.getMotorType(), config.getMotorsPerSide());

        sim = new DifferentialDrivetrainSim(
                motorModel,
                config.getGearRatio(),
                config.getTrackWidthMeters(),
                config.getWheelRadiusMeters(),
                config.getRobotMassKg(),
                config.getRobotMomentOfInertia(),
                VecBuilder.fill(0, 0, 0, 0, 0, 0, 0));
    }

    /**
     * Advances the simulation by {@code dtSeconds} using the commanded motor outputs.
     *
     * @param dtSeconds  timestep in seconds
     * @param leftOutput last commanded left motor output in the range [-1, 1]
     * @param rightOutput last commanded right motor output in the range [-1, 1]
     * @return chassis speeds inferred from the simulation state
     */
    public ChassisSpeeds update(double dtSeconds, double leftOutput, double rightOutput) {
        double leftVoltage = MathUtil.clamp(leftOutput, -1.0, 1.0) * config.getNominalVoltage();
        double rightVoltage = MathUtil.clamp(rightOutput, -1.0, 1.0) * config.getNominalVoltage();

        sim.setInputs(leftVoltage, rightVoltage);
        sim.update(dtSeconds);

        double leftPositionMeters = sim.getLeftPositionMeters();
        double rightPositionMeters = sim.getRightPositionMeters();
        double leftVelocityMetersPerSecond = sim.getLeftVelocityMetersPerSecond();
        double rightVelocityMetersPerSecond = sim.getRightVelocityMetersPerSecond();

        applyEncoderStates(leftEncoders, leftPositionMeters, leftVelocityMetersPerSecond);
        applyEncoderStates(rightEncoders, rightPositionMeters, rightVelocityMetersPerSecond);

        if (imu != null) {
            double angularRate = (rightVelocityMetersPerSecond - leftVelocityMetersPerSecond)
                    / config.getTrackWidthMeters();
            imu.setSimulatedHeading(sim.getHeading(), Rotation2d.fromRadians(angularRate));
        }

        double linearVelocity = (leftVelocityMetersPerSecond + rightVelocityMetersPerSecond) / 2.0;
        double angularVelocity = (rightVelocityMetersPerSecond - leftVelocityMetersPerSecond)
                / config.getTrackWidthMeters();

        return new ChassisSpeeds(linearVelocity, 0.0, angularVelocity);
    }

    public Pose2d getPose() {
        return sim.getPose();
    }

    public void resetPose(Pose2d pose) {
        sim.setPose(pose);
        applyEncoderStates(leftEncoders, 0, 0);
        applyEncoderStates(rightEncoders, 0, 0);
        if (imu != null) {
            imu.setSimulatedHeading(pose.getRotation(), Rotation2d.fromRadians(0));
        }
    }

    private static void applyEncoderStates(EncoderGroup group, double positionMeters, double velocityMetersPerSecond) {
        if (group == null) {
            return;
        }

        for (Encoder encoder : group.getEncoders()) {
            if (encoder == null) {
                continue;
            }

            double conversion = encoder.getConversion();
            if (!Double.isFinite(conversion) || Math.abs(conversion) < 1e-9) {
                conversion = 1.0;
            }

            double gearRatio = encoder.getGearRatio();
            if (!Double.isFinite(gearRatio) || Math.abs(gearRatio) < 1e-9) {
                gearRatio = 1.0;
            }

            double conversionOffset = encoder.getConversionOffset();
            double offset = encoder.getOffset();

            double wheelRotations = (positionMeters + conversionOffset) / conversion;
            double rawRotations = (wheelRotations + offset) / gearRatio;

            double wheelRotationRate = velocityMetersPerSecond / conversion;
            double rawVelocity = wheelRotationRate / gearRatio;

            encoder.setSimulatedState(rawRotations, rawVelocity);
        }
    }

    private static DCMotor resolveMotor(MotorSimType motorType, int motorsPerSide) {
        int count = Math.max(1, motorsPerSide);
        MotorSimType resolved = motorType != null ? motorType : Motor.CIM;
        return resolved.createSimMotor(count);
    }
}
