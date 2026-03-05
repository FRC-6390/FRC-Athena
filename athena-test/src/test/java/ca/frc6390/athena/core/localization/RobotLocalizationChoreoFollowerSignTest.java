package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class RobotLocalizationChoreoFollowerSignTest {

    @AfterEach
    void resetRobotTimeCache() {
        ca.frc6390.athena.core.RobotTime.resetNowSecondsForTest();
    }

    @Test
    void followerCommandsPositiveOmegaForPositiveHeadingError() throws Exception {
        TestFixture fixture = newFixture();

        fixture.localization.resetPose("field", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        fixture.follower.accept(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(20.0)),
                new ChassisSpeeds());

        ChassisSpeeds output = fixture.speeds.getInputSpeeds(RobotSpeeds.AUTO_SOURCE);
        assertTrue(
                output.omegaRadiansPerSecond > 0.0,
                "Positive heading error should command positive omega, got " + output.omegaRadiansPerSecond);
    }

    @Test
    void followerCommandsNegativeOmegaForNegativeHeadingError() throws Exception {
        TestFixture fixture = newFixture();

        fixture.localization.resetPose("field", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        fixture.follower.accept(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-20.0)),
                new ChassisSpeeds());

        ChassisSpeeds output = fixture.speeds.getInputSpeeds(RobotSpeeds.AUTO_SOURCE);
        assertTrue(
                output.omegaRadiansPerSecond < 0.0,
                "Negative heading error should command negative omega, got " + output.omegaRadiansPerSecond);
    }

    @Test
    void followerCommandsPositiveTranslationForPositiveFieldErrors() throws Exception {
        TestFixture fixture = newFixture();

        fixture.localization.resetPose("field", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        fixture.follower.accept(
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0.0)),
                new ChassisSpeeds());

        ChassisSpeeds output = fixture.speeds.getInputSpeeds(RobotSpeeds.AUTO_SOURCE);
        assertTrue(
                output.vxMetersPerSecond > 0.0,
                "Positive X error should command positive vx, got " + output.vxMetersPerSecond);
        assertTrue(
                output.vyMetersPerSecond > 0.0,
                "Positive Y error should command positive vy, got " + output.vyMetersPerSecond);
    }

    @Test
    void followerUsesCorrectFieldToRobotTransformAtNinetyDegrees() throws Exception {
        TestFixture fixture = newFixture();

        fixture.localization.resetPose("field", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0)));
        fixture.follower.accept(
                new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(90.0)),
                new ChassisSpeeds());

        ChassisSpeeds output = fixture.speeds.calculate(Rotation2d.fromDegrees(90.0));
        assertTrue(
                Math.abs(output.vxMetersPerSecond) < 1e-6,
                "At 90deg heading, field +X correction should map to near-zero robot vx, got "
                        + output.vxMetersPerSecond);
        assertTrue(
                output.vyMetersPerSecond < 0.0,
                "At 90deg heading, field +X correction should map to negative robot vy, got "
                        + output.vyMetersPerSecond);
    }

    @Test
    void followerWrapsHeadingErrorAcrossPlusMinusPiBoundary() throws Exception {
        TestFixture fixture = newFixture();

        fixture.localization.resetPose("field", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(179.0)));
        fixture.follower.accept(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-179.0)),
                new ChassisSpeeds());

        ChassisSpeeds output = fixture.speeds.getInputSpeeds(RobotSpeeds.AUTO_SOURCE);
        double wrappedError = MathUtil.inputModulus(-179.0 - 179.0, -180.0, 180.0);
        assertTrue(
                wrappedError > 0.0 && output.omegaRadiansPerSecond > 0.0,
                "Wraparound heading error should command positive omega across +pi/-pi boundary."
                        + " errorDeg=" + wrappedError + " omega=" + output.omegaRadiansPerSecond);
    }

    @Test
    void followerInvertsRotationFeedbackWhenRotationPidMarkedInverted() throws Exception {
        TestFixture fixture = newFixture(false, true);

        fixture.localization.resetPose("field", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        fixture.follower.accept(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(20.0)),
                new ChassisSpeeds());

        ChassisSpeeds output = fixture.speeds.getInputSpeeds(RobotSpeeds.AUTO_SOURCE);
        assertTrue(
                output.omegaRadiansPerSecond < 0.0,
                "Inverted rotation PID should command negative omega for positive heading error, got "
                        + output.omegaRadiansPerSecond);
    }

    @Test
    void followerInvertsTranslationFeedbackWhenTranslationPidMarkedInverted() throws Exception {
        TestFixture fixture = newFixture(true, false);

        fixture.localization.resetPose("field", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        fixture.follower.accept(
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0.0)),
                new ChassisSpeeds());

        ChassisSpeeds output = fixture.speeds.getInputSpeeds(RobotSpeeds.AUTO_SOURCE);
        assertTrue(
                output.vxMetersPerSecond < 0.0,
                "Inverted translation PID should command negative vx for positive X error, got "
                        + output.vxMetersPerSecond);
        assertTrue(
                output.vyMetersPerSecond < 0.0,
                "Inverted translation PID should command negative vy for positive Y error, got "
                        + output.vyMetersPerSecond);
    }

    @Test
    void followerTelemetryPublishesPathErrorAndFollowerLag() throws Exception {
        TestFixture fixture = newFixture();

        fixture.localization.resetPose("field", new Pose2d(0.2, -0.4, Rotation2d.fromDegrees(5.0)));
        setRobotTimeSeconds(10.00);
        fixture.follower.accept(
                new Pose2d(1.2, 0.6, Rotation2d.fromDegrees(15.0)),
                new ChassisSpeeds(1.5, -0.4, 0.8));

        assertEquals(1.0, readDoubleField(fixture.localization, "autoFollowerErrorXMeters"), 1e-6);
        assertEquals(1.0, readDoubleField(fixture.localization, "autoFollowerErrorYMeters"), 1e-6);
        assertEquals(
                Math.hypot(1.0, 1.0),
                readDoubleField(fixture.localization, "autoFollowerTranslationErrorMeters"),
                1e-6);
        assertEquals(10.0, readDoubleField(fixture.localization, "autoFollowerHeadingErrorDegrees"), 1e-6);

        setRobotTimeSeconds(10.02);
        fixture.follower.accept(
                new Pose2d(1.2, 0.6, Rotation2d.fromDegrees(15.0)),
                new ChassisSpeeds(1.5, -0.4, 0.8));

        double updateDtSeconds = readDoubleField(fixture.localization, "autoFollowerUpdateDtSeconds");
        double sourceAgeSeconds = readDoubleField(fixture.localization, "autoFollowerSourceAgeSeconds");
        double sourceEstimatedPeriodSeconds =
                readDoubleField(fixture.localization, "autoFollowerSourceEstimatedPeriodSeconds");
        assertTrue(updateDtSeconds >= 0.019 && updateDtSeconds <= 0.021,
                "expected 20ms follower dt telemetry, got " + updateDtSeconds);
        assertTrue(sourceAgeSeconds >= 0.019 && sourceAgeSeconds <= 0.021,
                "expected 20ms source-age telemetry, got " + sourceAgeSeconds);
        assertTrue(sourceEstimatedPeriodSeconds > 0.0,
                "expected positive source estimated period telemetry, got " + sourceEstimatedPeriodSeconds);
        assertTrue(readBooleanField(fixture.localization, "autoFollowerTelemetryValid"),
                "auto follower telemetry should be marked valid after follower updates.");
    }

    private static TestFixture newFixture() throws Exception {
        return newFixture(false, false);
    }

    private static TestFixture newFixture(boolean invertTranslation, boolean invertRotation) throws Exception {
        TestImu imu = new TestImu();
        imu.setVirtualAxis("field", Rotation2d.fromDegrees(0.0));
        imu.setVirtualAxis("driver", Rotation2d.fromDegrees(0.0));
        imu.setVirtualAxis("drift", Rotation2d.fromDegrees(0.0));

        RobotLocalizationConfig config = new RobotLocalizationConfig();
        config.config().autoPlannerPid(
                new HolonomicPidConstants(1.0, 0.0, 0.0, 0.0, invertTranslation),
                new HolonomicPidConstants(1.0, 0.0, 0.0, 0.0, invertRotation));
        RobotSpeeds speeds = new RobotSpeeds(4.5, Math.PI);

        RobotLocalization<DifferentialDriveWheelPositions> localization = new RobotLocalization<>(
                new DifferentialTestEstimatorFactory(0.62),
                config,
                speeds,
                imu,
                () -> new DifferentialDriveWheelPositions(0.0, 0.0));
        localization.configureChoreo(new SubsystemBase() {});

        AutoBackend backend = AutoBackends.forSource(RobotAuto.AutoSource.CHOREO)
                .orElseThrow(() -> new IllegalStateException("Expected CHOREO backend on test classpath."));
        BiConsumer<Pose2d, ChassisSpeeds> follower = extractFollower(backend);
        assertNotNull(follower, "Choreo backend follower should be configured.");

        return new TestFixture(localization, speeds, follower);
    }

    private static void setRobotTimeSeconds(double nowSeconds) throws Exception {
        Method updateNowSeconds = ca.frc6390.athena.core.RobotTime.class
                .getDeclaredMethod("updateNowSeconds", double.class);
        updateNowSeconds.setAccessible(true);
        updateNowSeconds.invoke(null, nowSeconds);
    }

    private static double readDoubleField(Object target, String name) throws Exception {
        Field field = target.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.getDouble(target);
    }

    private static boolean readBooleanField(Object target, String name) throws Exception {
        Field field = target.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.getBoolean(target);
    }

    @SuppressWarnings("unchecked")
    private static BiConsumer<Pose2d, ChassisSpeeds> extractFollower(AutoBackend backend) throws Exception {
        Field field = backend.getClass().getDeclaredField("follower");
        field.setAccessible(true);
        return (BiConsumer<Pose2d, ChassisSpeeds>) field.get(backend);
    }

    private record TestFixture(
            RobotLocalization<DifferentialDriveWheelPositions> localization,
            RobotSpeeds speeds,
            BiConsumer<Pose2d, ChassisSpeeds> follower) {
    }

    private static final class DifferentialTestEstimatorFactory
            implements PoseEstimatorFactory<DifferentialDriveWheelPositions> {
        private final DifferentialDriveKinematics kinematics;

        private DifferentialTestEstimatorFactory(double trackWidthMeters) {
            this.kinematics = new DifferentialDriveKinematics(trackWidthMeters);
        }

        @Override
        public PoseEstimator<DifferentialDriveWheelPositions> create2d(
                Pose2d startPose,
                Matrix<N3, N1> stateStdDevs,
                Matrix<N3, N1> visionStdDevs) {
            return new DifferentialDrivePoseEstimator(
                    kinematics,
                    startPose.getRotation(),
                    0.0,
                    0.0,
                    startPose,
                    stateStdDevs,
                    visionStdDevs);
        }

        @Override
        public PoseEstimator3d<DifferentialDriveWheelPositions> create3d(
                Pose3d startPose,
                Matrix<N4, N1> stateStdDevs,
                Matrix<N4, N1> visionStdDevs) {
            throw new UnsupportedOperationException("2D estimator only for this test.");
        }
    }

    private static final class TestImu implements Imu {
        private static final ImuType TYPE = () -> "test";
        private final ImuConfig config = ImuConfig.create(TYPE);
        private final Map<String, Rotation2d> virtualAxes = new HashMap<>();
        private Rotation2d yaw = new Rotation2d();

        @Override
        public Rotation2d getRoll() {
            return new Rotation2d();
        }

        @Override
        public Rotation2d getPitch() {
            return new Rotation2d();
        }

        @Override
        public Rotation2d getYaw() {
            return yaw;
        }

        @Override
        public void setInverted(boolean inverted) {
        }

        @Override
        public void setYaw(Rotation2d yaw) {
            this.yaw = yaw == null ? new Rotation2d() : yaw;
            setVirtualAxis("driver", this.yaw);
        }

        @Override
        public void addVirtualAxis(String name, Supplier<Rotation2d> supplier) {
            virtualAxes.put(name, supplier == null ? new Rotation2d() : supplier.get());
        }

        @Override
        public Rotation2d getVirtualAxis(String name) {
            return virtualAxes.getOrDefault(name, new Rotation2d());
        }

        @Override
        public void setVirtualAxis(String name, Rotation2d value) {
            virtualAxes.put(name, value == null ? new Rotation2d() : value);
        }

        @Override
        public ImuConfig getConfig() {
            return config;
        }

        @Override
        public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
            return node;
        }
    }
}
