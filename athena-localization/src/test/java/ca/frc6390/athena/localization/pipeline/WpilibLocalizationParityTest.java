package ca.frc6390.athena.localization.pipeline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModuleModel;
import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class WpilibLocalizationParityTest {
    private static final double PARITY_TOLERANCE = 1.0e-8;
    private static final MeasurementStdDevs STATE_STD_DEVS = MeasurementStdDevs.of(0.08, 0.08, 0.04);
    private static final MeasurementStdDevs DEFAULT_VISION_STD_DEVS = MeasurementStdDevs.of(0.7, 0.7, 0.4);

    @Test
    void odometryOnlyMatchesWpilibFrameByFrame() {
        ParityRig rig = new ParityRig();

        rig.step(0.00, distances(0.0, 0.0, 0.0, 0.0), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.step(0.02, distances(0.10, 0.10, 0.10, 0.10), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.step(0.04, distances(0.20, 0.20, 0.20, 0.20), angles(0.25, 0.25, 0.25, 0.25), 0.0);
        rig.step(0.06, distances(0.25, 0.35, 0.25, 0.35), angles(-0.125, 0.125, 0.125, -0.125), 12.0);
        rig.step(0.08, distances(0.31, 0.43, 0.30, 0.44), angles(-0.12, 0.13, 0.12, -0.13), 35.0);
        rig.step(0.10, distances(0.40, 0.52, 0.39, 0.53), angles(0.5, 0.5, 0.5, 0.5), 179.0);
        rig.step(0.12, distances(0.48, 0.61, 0.47, 0.62), angles(0.5, 0.5, 0.5, 0.5), -179.0);
    }

    @Test
    void immediateAndDelayedVisionMatchWpilibFrameByFrame() {
        ParityRig rig = new ParityRig();
        Object frontCamera = new Object();
        Object rearCamera = new Object();

        rig.step(0.0, distances(0.0, 0.0, 0.0, 0.0), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.step(0.5, distances(0.5, 0.5, 0.5, 0.5), angles(0.0, 0.0, 0.0, 0.0), 0.0,
                vision(0.46, 0.03, 0.01, 0.5, MeasurementStdDevs.of(0.15, 0.18, 0.12), frontCamera));
        rig.step(1.0, distances(1.0, 1.0, 1.0, 1.0), angles(0.0, 0.0, 0.0, 0.0), 2.0);
        rig.step(1.5, distances(1.5, 1.5, 1.5, 1.5), angles(0.0, 0.0, 0.0, 0.0), 4.0,
                vision(0.92, -0.04, 0.02, 0.9, MeasurementStdDevs.of(0.10, 0.12, 0.08), rearCamera));
        rig.step(2.0, distances(2.0, 2.0, 2.0, 2.0), angles(0.0, 0.0, 0.0, 0.0), 6.0,
                vision(1.96, 0.08, 0.10, 2.0, MeasurementStdDevs.of(0.25, 0.20, 0.15), frontCamera));
        rig.step(2.5, distances(2.5, 2.5, 2.5, 2.5), angles(0.0, 0.0, 0.0, 0.0), 8.0);
    }

    @Test
    void resetMatchesFreshWpilibEstimatorAndDoesNotReuseOldWheelTravel() {
        ParityRig rig = new ParityRig();

        rig.step(0.0, distances(0.0, 0.0, 0.0, 0.0), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.step(0.5, distances(0.5, 0.5, 0.5, 0.5), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.reset(new PoseSnapshot(4.0, 2.0, Math.toRadians(90.0)));
        rig.step(1.0, distances(0.9, 0.9, 0.9, 0.9), angles(0.0, 0.0, 0.0, 0.0), 15.0);
        rig.step(1.5, distances(1.2, 1.2, 1.2, 1.2), angles(0.0, 0.0, 0.0, 0.0), 15.0);
    }

    private static double[] distances(double frontLeft, double frontRight, double backLeft, double backRight) {
        return new double[] {frontLeft, frontRight, backLeft, backRight};
    }

    private static double[] angles(double frontLeft, double frontRight, double backLeft, double backRight) {
        return new double[] {frontLeft, frontRight, backLeft, backRight};
    }

    private static VisionSample vision(
            double xMeters,
            double yMeters,
            double headingRadians,
            double timestampSeconds,
            MeasurementStdDevs stdDevs,
            Object source) {
        return new VisionSample(
                new PoseSnapshot(xMeters, yMeters, headingRadians),
                RobotVelocity.zero(),
                timestampSeconds,
                0.02,
                0.05,
                2,
                2.0,
                stdDevs,
                source);
    }

    private static final class ParityRig {
        private final Map<Integer, Double> driveRotations = initializedMap();
        private final Map<Integer, Double> moduleAngles = initializedMap();
        private final TestImu imu = new TestImu();
        private final SwerveModule frontLeft = module(1, 11, 21);
        private final SwerveModule frontRight = module(2, 12, 22);
        private final SwerveModule backLeft = module(3, 13, 23);
        private final SwerveModule backRight = module(4, 14, 24);
        private final SwerveKinematics athenaKinematics = SwerveKinematics.rectangular(
                0.5, 0.5, 5.0, frontLeft, frontRight, backLeft, backRight);
        private final SwerveDriveKinematics wpilibKinematics = new SwerveDriveKinematics(
                new Translation2d(0.25, 0.25),
                new Translation2d(0.25, -0.25),
                new Translation2d(-0.25, 0.25),
                new Translation2d(-0.25, -0.25));
        private final MutablePoseSignal visionSource = new MutablePoseSignal();
        private final Localization visionStage = Localizations.filter().input(visionSource);
        private final SwerveOdometry odometry = athenaKinematics.odometry(imu);
        private final Localization athena = Localizations.kalman()
                .input(odometry, visionStage)
                .stateStdDevs(STATE_STD_DEVS.xMeters(), STATE_STD_DEVS.headingRadians())
                .defaultVisionStdDevs(DEFAULT_VISION_STD_DEVS.xMeters(), DEFAULT_VISION_STD_DEVS.headingRadians());
        private final ActionContext context = new ActionContext() {
            @Override
            public EncoderHandle encoder(EncoderDevice device) {
                if (device.source() instanceof EncoderDevice.EncoderSource.IntegratedMotor integrated) {
                    return handle(device, driveRotations, integrated.motor().id());
                }
                return handle(device, moduleAngles, device.id());
            }
        };
        private SwerveDrivePoseEstimator wpilib;
        private PoseSnapshot pendingReset;

        private void step(
                double timestampSeconds,
                double[] distancesMeters,
                double[] angleRotations,
                double yawDegrees,
                VisionSample... vision) {
            setModuleInputs(distancesMeters, angleRotations);
            imu.yawDegrees = yawDegrees;
            visionSource.measurements = List.of(vision);
            athena.refresh(context, timestampSeconds, 0.02);

            Rotation2d heading = new Rotation2d(odometry.headingRadians());
            SwerveModulePosition[] positions = wpilibPositions();
            if (wpilib == null || pendingReset != null) {
                PoseSnapshot initial = pendingReset == null ? odometry.pose() : pendingReset;
                wpilib = new SwerveDrivePoseEstimator(
                        wpilibKinematics,
                        heading,
                        positions,
                        toWpilib(initial),
                        VecBuilder.fill(
                                STATE_STD_DEVS.xMeters(),
                                STATE_STD_DEVS.yMeters(),
                                STATE_STD_DEVS.headingRadians()),
                        VecBuilder.fill(
                                DEFAULT_VISION_STD_DEVS.xMeters(),
                                DEFAULT_VISION_STD_DEVS.yMeters(),
                                DEFAULT_VISION_STD_DEVS.headingRadians()));
                pendingReset = null;
            }
            wpilib.updateWithTime(timestampSeconds, heading, positions);
            for (VisionSample sample : vision) {
                wpilib.addVisionMeasurement(
                        toWpilib(sample.pose()),
                        sample.timestampSeconds(),
                        VecBuilder.fill(
                                sample.stdDevs().xMeters(),
                                sample.stdDevs().yMeters(),
                                sample.stdDevs().headingRadians()));
            }
            assertPoseParity(timestampSeconds, athena.pose(), wpilib.getEstimatedPosition());
        }

        private void reset(PoseSnapshot pose) {
            athena.reset(pose).apply(context);
            pendingReset = pose;
        }

        private void setModuleInputs(double[] distancesMeters, double[] angleRotations) {
            if (distancesMeters.length != 4 || angleRotations.length != 4) {
                throw new IllegalArgumentException("Expected four module values.");
            }
            for (int index = 0; index < 4; index++) {
                driveRotations.put(index + 1, distancesMeters[index] * 2.0);
                moduleAngles.put(index + 21, angleRotations[index]);
            }
        }

        private SwerveModulePosition[] wpilibPositions() {
            return odometry.modulePositions().stream()
                    .map(position -> new SwerveModulePosition(
                            position.distanceMeters(),
                            Rotation2d.fromRotations(position.angleRotations())))
                    .toArray(SwerveModulePosition[]::new);
        }

        private static Map<Integer, Double> initializedMap() {
            Map<Integer, Double> values = new HashMap<>();
            for (int id = 1; id <= 24; id++) {
                values.put(id, 0.0);
            }
            return values;
        }

        private static EncoderHandle handle(EncoderDevice device, Map<Integer, Double> values, int id) {
            return new EncoderHandle() {
                @Override
                public EncoderDevice device() {
                    return device;
                }

                @Override
                public double positionRotations() {
                    return values.get(id);
                }

                @Override
                public double absolutePositionRotations() {
                    return values.get(id);
                }

                @Override
                public double velocityRotationsPerSecond() {
                    return 0.0;
                }
            };
        }

        private static SwerveModule module(int driveId, int steerId, int angleId) {
            TestModule module = new TestModule();
            module.drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId));
            module.steer.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, steerId));
            module.angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, angleId).units(EncoderUnit.ROTATIONS));
            return module;
        }
    }

    private static Pose2d toWpilib(PoseSnapshot pose) {
        return new Pose2d(pose.xMeters(), pose.yMeters(), new Rotation2d(pose.headingRadians()));
    }

    private static void assertPoseParity(double timestampSeconds, PoseSnapshot athena, Pose2d wpilib) {
        assertEquals(wpilib.getX(), athena.xMeters(), PARITY_TOLERANCE, "x at t=" + timestampSeconds);
        assertEquals(wpilib.getY(), athena.yMeters(), PARITY_TOLERANCE, "y at t=" + timestampSeconds);
        double headingError = Math.IEEEremainder(
                athena.headingRadians() - wpilib.getRotation().getRadians(),
                Math.PI * 2.0);
        assertEquals(0.0, headingError, PARITY_TOLERANCE, "heading at t=" + timestampSeconds);
    }

    private static final class MutablePoseSignal implements PoseSignal {
        private List<Measurement> measurements = List.of();

        @Override
        public List<Measurement> measurements() {
            return measurements;
        }
    }

    private record VisionSample(
            PoseSnapshot pose,
            RobotVelocity speeds,
            double timestampSeconds,
            double latencySeconds,
            double ambiguity,
            int targetCount,
            double averageTargetDistanceMeters,
            MeasurementStdDevs stdDevs,
            Object source) implements PoseMeasurementSample {
    }

    private static final class TestModule extends SwerveModule {
        private TestModule() {
            super(SwerveModuleModel.custom(2.0, 1.0, 1.0 / Math.PI));
        }
    }

    private static final class TestImu implements ImuSource {
        private double yawDegrees;

        @Override public double yawDegrees() { return yawDegrees; }
        @Override public double pitchDegrees() { return 0.0; }
        @Override public double rollDegrees() { return 0.0; }
        @Override public double angleDegrees() { return yawDegrees; }
        @Override public double yawRateDegreesPerSecond() { return 0.0; }
        @Override public double linearAccelerationXG() { return 0.0; }
        @Override public double linearAccelerationYG() { return 0.0; }
        @Override public double linearAccelerationZG() { return 0.0; }
        @Override public void applyYaw(ActionContext context, double yawDegrees) { this.yawDegrees = yawDegrees; }
    }
}
