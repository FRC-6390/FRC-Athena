package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.RobotSpeeds;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

final class RobotLocalizationSlipSolverTest {

    @Test
    void moderateImuSpeedMismatchActivatesSlipAssistAndDampsTranslation() throws Exception {
        TestImu imu = new TestImu();
        imu.xSpeedMetersPerSecond = 0.30;
        imu.ySpeedMetersPerSecond = 0.0;

        RobotSpeeds robotSpeeds = new RobotSpeeds(4.5, Math.PI);
        robotSpeeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 0.0);

        MutableWheelPositions wheelPositions = new MutableWheelPositions();
        RobotLocalization<DifferentialDriveWheelPositions> localization = new RobotLocalization<>(
                new DifferentialTestEstimatorFactory(0.62),
                new RobotLocalizationConfig(),
                robotSpeeds,
                imu,
                wheelPositions::snapshot);

        setField(localization, "lastUpdateTimestamp", 0.0);
        setField(localization, "lastFieldPoseForSlip", new Pose2d());
        setField(localization, "lastFieldVelocityForSlip", new Translation2d());
        setField(localization, "fieldPose", new Pose2d(0.20, 0.0, new Rotation2d()));

        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.10);
        Map<String, Object> summary = localization.getDiagnosticsSummary();
        String mode = String.valueOf(summary.get("slipMode"));
        assertTrue(
                "TRANSIENT".equals(mode) || "SLIP".equals(mode),
                "expected moderate contact mismatch to trigger active slip handling");

        setField(localization, "lastRawEstimatorPoseForSlipFusion", new Pose2d());
        setField(localization, "lastCorrectedPoseForSlipFusion", new Pose2d());
        setField(localization, "lastSlipFusionTimestamp", 0.0);

        Translation2d corrected = (Translation2d) invoke(
                localization,
                "computeSlipAwareFieldTranslation",
                new Class<?>[] { Pose2d.class, Rotation2d.class, double.class },
                new Pose2d(0.20, 0.0, new Rotation2d()),
                new Rotation2d(),
                0.10);

        assertTrue(
                corrected.getX() < 0.16,
                "expected slip assist to reduce forward translation under wheel/IMU mismatch");
    }

    @Test
    void wallContactAfterMotionWithImuZeroTriggersSlipAndLocksTranslation() throws Exception {
        TestImu imu = new TestImu();
        imu.xSpeedMetersPerSecond = 1.0;
        imu.ySpeedMetersPerSecond = 0.0;

        RobotSpeeds robotSpeeds = new RobotSpeeds(4.5, Math.PI);
        robotSpeeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 0.0);

        MutableWheelPositions wheelPositions = new MutableWheelPositions();
        RobotLocalization<DifferentialDriveWheelPositions> localization = new RobotLocalization<>(
                new DifferentialTestEstimatorFactory(0.62),
                new RobotLocalizationConfig(),
                robotSpeeds,
                imu,
                wheelPositions::snapshot);

        setField(localization, "lastUpdateTimestamp", 0.0);
        setField(localization, "lastFieldPoseForSlip", new Pose2d());
        setField(localization, "lastFieldVelocityForSlip", new Translation2d());

        setField(localization, "fieldPose", new Pose2d(0.20, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.10);

        imu.xSpeedMetersPerSecond = 0.0;
        setField(localization, "fieldPose", new Pose2d(0.40, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.20);
        Map<String, Object> summary = localization.getDiagnosticsSummary();
        assertEquals("SLIP", summary.get("slipMode"),
                "expected wall-contact mismatch to escalate to SLIP once IMU linear signal had been observed");

        setField(localization, "lastRawEstimatorPoseForSlipFusion", new Pose2d(0.20, 0.0, new Rotation2d()));
        setField(localization, "lastCorrectedPoseForSlipFusion", new Pose2d(0.20, 0.0, new Rotation2d()));
        setField(localization, "lastSlipFusionTimestamp", 0.10);

        Translation2d corrected = (Translation2d) invoke(
                localization,
                "computeSlipAwareFieldTranslation",
                new Class<?>[] { Pose2d.class, Rotation2d.class, double.class },
                new Pose2d(0.40, 0.0, new Rotation2d()),
                new Rotation2d(),
                0.20);

        assertTrue(
                corrected.getX() < 0.24,
                "expected wall-contact slip fusion to hold translation near prior corrected pose");
    }

    @Test
    void wallContactEvidenceTrackingPersistsWhenCorrectedOdomTemporarilyDrops() throws Exception {
        TestImu imu = new TestImu();
        imu.xSpeedMetersPerSecond = 1.0;
        imu.ySpeedMetersPerSecond = 0.0;

        RobotSpeeds robotSpeeds = new RobotSpeeds(4.5, Math.PI);
        robotSpeeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 0.0);

        MutableWheelPositions wheelPositions = new MutableWheelPositions();
        RobotLocalization<DifferentialDriveWheelPositions> localization = new RobotLocalization<>(
                new DifferentialTestEstimatorFactory(0.62),
                new RobotLocalizationConfig(),
                robotSpeeds,
                imu,
                wheelPositions::snapshot);

        setField(localization, "lastUpdateTimestamp", 0.0);
        setField(localization, "lastFieldPoseForSlip", new Pose2d());
        setField(localization, "lastFieldVelocityForSlip", new Translation2d());

        setField(localization, "fieldPose", new Pose2d(0.20, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.10);

        imu.xSpeedMetersPerSecond = 0.0;
        setField(localization, "fieldPose", new Pose2d(0.40, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.20);

        setField(localization, "fieldPose", new Pose2d(0.42, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.30);

        double contactComponent = (double) getField(localization, "slipContactComponent");
        assertTrue(
                contactComponent >= 0.40,
                "expected recent wall-contact mismatch evidence to remain tracked across cycles");

        setField(localization, "lastRawEstimatorPoseForSlipFusion", new Pose2d(0.40, 0.0, new Rotation2d()));
        setField(localization, "lastCorrectedPoseForSlipFusion", new Pose2d(0.22, 0.0, new Rotation2d()));
        setField(localization, "lastSlipFusionTimestamp", 0.20);

        Translation2d corrected = (Translation2d) invoke(
                localization,
                "computeSlipAwareFieldTranslation",
                new Class<?>[] { Pose2d.class, Rotation2d.class, double.class },
                new Pose2d(0.60, 0.0, new Rotation2d()),
                new Rotation2d(),
                0.30);

        assertTrue(
                corrected.getX() < 0.28,
                "expected tracked contact evidence to keep translation from pushing through a static obstacle");
    }

    @Test
    void quickDirectionChangeWheelspinSuppressesReversalTranslationSpike() throws Exception {
        TestImu imu = new TestImu();
        imu.xSpeedMetersPerSecond = 1.0;
        imu.ySpeedMetersPerSecond = 0.0;

        RobotSpeeds robotSpeeds = new RobotSpeeds(4.5, Math.PI);
        robotSpeeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 0.0);

        MutableWheelPositions wheelPositions = new MutableWheelPositions();
        RobotLocalization<DifferentialDriveWheelPositions> localization = new RobotLocalization<>(
                new DifferentialTestEstimatorFactory(0.62),
                new RobotLocalizationConfig(),
                robotSpeeds,
                imu,
                wheelPositions::snapshot);

        setField(localization, "lastUpdateTimestamp", 0.0);
        setField(localization, "lastFieldPoseForSlip", new Pose2d());
        setField(localization, "lastFieldVelocityForSlip", new Translation2d());

        setField(localization, "fieldPose", new Pose2d(0.20, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.10);

        robotSpeeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, -2.0, 0.0, 0.0);
        imu.xSpeedMetersPerSecond = 0.0;
        setField(localization, "fieldPose", new Pose2d(0.00, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.20);
        Map<String, Object> summary = localization.getDiagnosticsSummary();
        assertEquals("SLIP", summary.get("slipMode"),
                "expected aggressive direction reversal with low IMU motion to trigger SLIP handling");

        setField(localization, "lastRawEstimatorPoseForSlipFusion", new Pose2d(0.20, 0.0, new Rotation2d()));
        setField(localization, "lastCorrectedPoseForSlipFusion", new Pose2d(0.20, 0.0, new Rotation2d()));
        setField(localization, "lastSlipFusionTimestamp", 0.10);

        Translation2d corrected = (Translation2d) invoke(
                localization,
                "computeSlipAwareFieldTranslation",
                new Class<?>[] { Pose2d.class, Rotation2d.class, double.class },
                new Pose2d(0.00, 0.0, new Rotation2d()),
                new Rotation2d(),
                0.20);

        assertTrue(
                corrected.getX() > 0.15,
                "expected slip fusion to reject the instantaneous reversal jump from wheelspin");
    }

    @Test
    void reversalSlipEvidenceTrackingPreventsImmediateClearOnSingleCalmCycle() throws Exception {
        TestImu imu = new TestImu();
        imu.xSpeedMetersPerSecond = 1.0;
        imu.ySpeedMetersPerSecond = 0.0;

        RobotSpeeds robotSpeeds = new RobotSpeeds(4.5, Math.PI);
        robotSpeeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 0.0);

        MutableWheelPositions wheelPositions = new MutableWheelPositions();
        RobotLocalization<DifferentialDriveWheelPositions> localization = new RobotLocalization<>(
                new DifferentialTestEstimatorFactory(0.62),
                new RobotLocalizationConfig(),
                robotSpeeds,
                imu,
                wheelPositions::snapshot);

        setField(localization, "lastUpdateTimestamp", 0.0);
        setField(localization, "lastFieldPoseForSlip", new Pose2d());
        setField(localization, "lastFieldVelocityForSlip", new Translation2d());

        setField(localization, "fieldPose", new Pose2d(0.20, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.10);

        robotSpeeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, -2.0, 0.0, 0.0);
        imu.xSpeedMetersPerSecond = 0.0;
        setField(localization, "fieldPose", new Pose2d(0.00, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.20);
        Map<String, Object> summary = localization.getDiagnosticsSummary();
        assertEquals("SLIP", summary.get("slipMode"),
                "expected reversal mismatch to enter SLIP before persistence tracking is evaluated");

        robotSpeeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 0.0, 0.0, 0.0);
        setField(localization, "fieldPose", new Pose2d(0.00, 0.0, new Rotation2d()));
        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.30);
        Map<String, Object> afterSingleCalmCycle = localization.getDiagnosticsSummary();
        assertEquals("SLIP", afterSingleCalmCycle.get("slipMode"),
                "expected SLIP mode to remain active while recent evidence is still elevated");
        assertTrue(
                Boolean.TRUE.equals(afterSingleCalmCycle.get("slipActive")),
                "expected slipActive to remain true while evidence tracking still indicates slip");
    }

    @Test
    void highSpeedRubbingUnderSpeedTriggersSlipAssistAndDampsOdom() throws Exception {
        TestImu imu = new TestImu();
        imu.xSpeedMetersPerSecond = 2.4;
        imu.ySpeedMetersPerSecond = 0.0;

        RobotSpeeds robotSpeeds = new RobotSpeeds(5.5, Math.PI);
        robotSpeeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 4.0, 0.0, 0.0);

        MutableWheelPositions wheelPositions = new MutableWheelPositions();
        RobotLocalization<DifferentialDriveWheelPositions> localization = new RobotLocalization<>(
                new DifferentialTestEstimatorFactory(0.62),
                new RobotLocalizationConfig(),
                robotSpeeds,
                imu,
                wheelPositions::snapshot);

        setField(localization, "lastUpdateTimestamp", 0.0);
        setField(localization, "lastFieldPoseForSlip", new Pose2d());
        setField(localization, "lastFieldVelocityForSlip", new Translation2d());
        setField(localization, "fieldPose", new Pose2d(0.40, 0.0, new Rotation2d()));

        invoke(localization, "updateSlipState", new Class<?>[] { double.class }, 0.10);
        Map<String, Object> summary = localization.getDiagnosticsSummary();
        assertEquals("TRANSIENT", summary.get("slipMode"),
                "expected under-speed at high commanded velocity to trigger transient slip assist");

        setField(localization, "lastRawEstimatorPoseForSlipFusion", new Pose2d());
        setField(localization, "lastCorrectedPoseForSlipFusion", new Pose2d());
        setField(localization, "lastSlipFusionTimestamp", 0.0);

        Translation2d corrected = (Translation2d) invoke(
                localization,
                "computeSlipAwareFieldTranslation",
                new Class<?>[] { Pose2d.class, Rotation2d.class, double.class },
                new Pose2d(0.40, 0.0, new Rotation2d()),
                new Rotation2d(),
                0.10);

        assertTrue(
                corrected.getX() < 0.36,
                "expected slip assist to damp raw odometry when fast motion is slower than expected");
    }

    private static void setField(Object target, String fieldName, Object value) throws Exception {
        Field field = RobotLocalization.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(target, value);
    }

    private static Object getField(Object target, String fieldName) throws Exception {
        Field field = RobotLocalization.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(target);
    }

    private static Object invoke(Object target, String name, Class<?>[] parameterTypes, Object... args) throws Exception {
        Method method = RobotLocalization.class.getDeclaredMethod(name, parameterTypes);
        method.setAccessible(true);
        return method.invoke(target, args);
    }

    private static final class MutableWheelPositions {
        double leftMeters;
        double rightMeters;

        DifferentialDriveWheelPositions snapshot() {
            return new DifferentialDriveWheelPositions(leftMeters, rightMeters);
        }
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
        private boolean inverted;
        private double xSpeedMetersPerSecond;
        private double ySpeedMetersPerSecond;

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
            this.inverted = inverted;
        }

        @Override
        public boolean isInverted() {
            return inverted;
        }

        @Override
        public double getXSpeedMetersPerSecond() {
            return xSpeedMetersPerSecond;
        }

        @Override
        public double getYSpeedMetersPerSecond() {
            return ySpeedMetersPerSecond;
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
        public void setYaw(Rotation2d yaw) {
            this.yaw = yaw == null ? new Rotation2d() : yaw;
            setVirtualAxis("driver", this.yaw);
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
