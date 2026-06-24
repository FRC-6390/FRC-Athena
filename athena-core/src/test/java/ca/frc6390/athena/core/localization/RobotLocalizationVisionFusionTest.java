package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera.CameraSoftware;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.StructSubscriber;
import org.junit.jupiter.api.Test;

final class RobotLocalizationVisionFusionTest {

    @Test
    void cameraPoseIsFusedAgainstOdometryEstimate() throws Exception {
        TestImu imu = new TestImu();
        MutableWheelPositions wheelPositions = new MutableWheelPositions();

        RobotLocalization<DifferentialDriveWheelPositions> visionLocalization = createLocalization(
                RobotLocalizationConfig.create()
                        .estimation(e -> e.visionEnabled(true).vision(0.02, 0.02, 0.001))
                        .backendConfig(b -> b.visionFusionMaxSeparationSeconds(0.5))
                        .backendConfig(b -> b.slipDetectionEnabled(false)),
                new RobotSpeeds(4.5, Math.PI),
                imu,
                wheelPositions::snapshot);
        RobotLocalization<DifferentialDriveWheelPositions> odomOnlyLocalization = createLocalization(
                RobotLocalizationConfig.create().estimation(e -> e.visionEnabled(false)),
                new RobotSpeeds(4.5, Math.PI),
                new TestImu(),
                wheelPositions::snapshot);

        AtomicBoolean cameraHasTargets = new AtomicBoolean(false);
        AtomicReference<Pose2d> cameraPose = new AtomicReference<>(new Pose2d());
        RobotVision vision = createRobotVision(cameraHasTargets, cameraPose, 0.01);
        visionLocalization.setRobotVision(vision);

        visionLocalization.setLastUpdateTimestampForTest(-1.0);
        odomOnlyLocalization.setLastUpdateTimestampForTest(-1.0);

        visionLocalization.periodic();
        odomOnlyLocalization.periodic();

        wheelPositions.leftMeters = 2.0;
        wheelPositions.rightMeters = 2.0;
        visionLocalization.periodic();
        odomOnlyLocalization.periodic();

        Pose2d odomOnlyPose = odomOnlyLocalization.getFieldPose();
        assertTrue(odomOnlyPose.getX() > 0.5,
                "odometry-only path should advance when wheel positions change");
        Pose2d visionPoseBeforeCamera = visionLocalization.getFieldPose();

        cameraHasTargets.set(true);
        cameraPose.set(new Pose2d(4.0, 0.0, Rotation2d.fromDegrees(0.0)));
        visionLocalization.periodic();
        odomOnlyLocalization.periodic();

        Pose2d visionReferencePose = new Pose2d(4.0, 0.0, Rotation2d.fromDegrees(0.0));
        Pose2d visionPose = visionLocalization.getFieldPose();
        double visionDistanceToVision = visionPose.getTranslation().getDistance(
                visionReferencePose.getTranslation());

        assertTrue(
                visionPose.getTranslation().getDistance(visionPoseBeforeCamera.getTranslation()) > 0.1,
                "vision input should perturb pose estimate on the next periodic cycle");
        assertTrue(visionDistanceToVision <= 3.0,
                "vision enabled estimate should remain near the camera sample");
        assertTrue(
                visionPose.getTranslation().getDistance(odomOnlyPose.getTranslation()) > 0.1,
                "vision-enabled and odometry-only trajectories should diverge after vision update");

        Pose2d finalVisionPose = visionLocalization.getFieldPose();
        Pose2d finalOdomPose = odomOnlyLocalization.getFieldPose();
        assertTrue(finalVisionPose.getY() < 1.5 && finalVisionPose.getY() > -1.5);
        assertTrue(Math.hypot(finalOdomPose.getX(), finalOdomPose.getY()) > 0.5,
                "odometry-only estimate should remain non-trivial after movement");
    }

    @Test
    void cameraPoseAndEstimatedPoseTopicsArePublished() throws Exception {
        RobotNetworkTables networkTables = new RobotNetworkTables();
        networkTables.publishConfig();
        networkTables.enable(RobotNetworkTables.Flag.LOCALIZATION_FIELD_WIDGET);
        networkTables.enable(RobotNetworkTables.Flag.VISION_CAMERA_WIDGETS);
        networkTables.setAsyncPublishingEnabled(false);

        RobotNetworkTables.Node visionCameraConfig = networkTables.root()
                .child("NetworkTableConfig")
                .child("Vision")
                .child("Cameras")
                .child("vision-fusion-widgets");
        NetworkTableEntry showCameraPoseEntry = visionCameraConfig.entry("ShowCameraPose");
        showCameraPoseEntry.setBoolean(true);
        NetworkTableEntry showEstimatedPoseEntry = visionCameraConfig.entry("ShowEstimatedPose");
        showEstimatedPoseEntry.setBoolean(true);
        NetworkTableEntry showTagLinesEntry = visionCameraConfig.entry("ShowTagLines");
        showTagLinesEntry.setBoolean(true);

        TestImu imu = new TestImu();
        MutableWheelPositions wheelPositions = new MutableWheelPositions();
        RobotLocalization<DifferentialDriveWheelPositions> localization = createLocalization(
                RobotLocalizationConfig.create()
                        .estimation(e -> e.visionEnabled(true).vision(0.02, 0.02, 0.001))
                        .backendConfig(b -> b.visionFusionMaxSeparationSeconds(0.5))
                        .backendConfig(b -> b.slipDetectionEnabled(false)),
                new RobotSpeeds(4.5, Math.PI),
                imu,
                wheelPositions::snapshot);
        localization.attachRobotNetworkTables(networkTables);

        String cameraKey = "vision-fusion-widgets";
        AtomicBoolean cameraHasTargets = new AtomicBoolean(false);
        AtomicReference<Pose2d> cameraPose = new AtomicReference<>(new Pose2d());
        RobotVision vision = createRobotVision(cameraHasTargets, cameraPose, 0.01, cameraKey);
        localization.setRobotVision(vision);

        StructSubscriber<Pose2d> cameraPoseSubscriber = NetworkTableInstance.getDefault()
                .getStructTopic("/Athena/Vision/Cameras/" + cameraKey + "/CameraPose", Pose2d.struct)
                .subscribe(new Pose2d());
        StructSubscriber<Pose2d> estimatedPoseSubscriber = NetworkTableInstance.getDefault()
                .getStructTopic("/Athena/Localization/Cameras/" + cameraKey + "/EstimatedPose", Pose2d.struct)
                .subscribe(new Pose2d());

        Pose2d initialPublishedCameraPose = cameraPoseSubscriber.get();
        Pose2d initialPublishedEstimatedPose = estimatedPoseSubscriber.get();
        assertEquals(0.0, initialPublishedCameraPose.getX(), 1e-9);
        assertEquals(0.0, initialPublishedCameraPose.getY(), 1e-9);
        assertEquals(0.0, initialPublishedEstimatedPose.getX(), 1e-9);

        localization.setLastUpdateTimestampForTest(-1.0);
        wheelPositions.leftMeters = 1.8;
        wheelPositions.rightMeters = 1.8;
        localization.periodic();

        cameraHasTargets.set(true);
        cameraPose.set(new Pose2d(4.0, 0.3, Rotation2d.fromDegrees(5.0)));
        localization.periodic();

        Pose2d publishedCameraPose = cameraPoseSubscriber.get();
        Pose2d publishedEstimatedPose = estimatedPoseSubscriber.get();
        assertTrue(publishedCameraPose.getX() > 0.5, "camera pose topic should move with robot field pose");
        assertTrue(Math.abs(publishedCameraPose.getY()) < 0.7, "camera pose topic should not drift wildly");
        assertTrue(publishedEstimatedPose.getX() > 0.5, "estimated pose topic should publish a camera-based estimate");
    }

    @Test
    void staticRobotTracksVisionWhenNoOdometry() throws Exception {
        TestImu imu = new TestImu();
        MutableWheelPositions wheelPositions = new MutableWheelPositions();

        RobotLocalization<DifferentialDriveWheelPositions> localization = createLocalization(
                RobotLocalizationConfig.create()
                        .estimation(e -> e.visionEnabled(true).vision(0.015, 0.015, 0.002))
                        .backendConfig(b -> b
                                .slipDetectionEnabled(false)
                                .poseJumpMeters(10.0)
                                .visionFusionMaxSeparationSeconds(0.5)),
                new RobotSpeeds(4.5, Math.PI),
                imu,
                wheelPositions::snapshot);

        AtomicBoolean cameraHasTargets = new AtomicBoolean(false);
        AtomicReference<Pose2d> cameraPose = new AtomicReference<>(new Pose2d());
        RobotVision vision = createRobotVision(cameraHasTargets, cameraPose, 0.01);
        localization.setRobotVision(vision);

        localization.setLastUpdateTimestampForTest(-1.0);
        localization.periodic();

        cameraHasTargets.set(true);
        cameraPose.set(new Pose2d(4.0, 0.5, Rotation2d.fromDegrees(12.0)));
        localization.periodic();

        Pose2d fusedPose = localization.getFieldPose();
        Pose2d measuredVisionPose = new Pose2d(4.0, 0.5, Rotation2d.fromDegrees(12.0));
        double fusedDistanceToVision = fusedPose.getTranslation().getDistance(measuredVisionPose.getTranslation());
        double noMeasurementDistanceToVision = measuredVisionPose.getTranslation().getDistance(new Translation2d());

        assertEquals(0.0, wheelPositions.leftMeters, 1e-9);
        assertEquals(0.0, wheelPositions.rightMeters, 1e-9);
        assertTrue(fusedDistanceToVision < noMeasurementDistanceToVision,
                "vision input should improve pose estimate when odometry is static");
        assertTrue(fusedPose.getX() > 0.75,
                "vision estimate should move the robot pose off the origin");
        assertTrue(fusedPose.getX() < 4.2);
        assertTrue(fusedPose.getY() > 0.0 && fusedPose.getY() < 1.0);
    }

    private static RobotVision createRobotVision(
            AtomicBoolean hasTargets,
            AtomicReference<Pose2d> pose,
            double latencySeconds) throws Exception {
        return createRobotVision(hasTargets, pose, latencySeconds, "vision-fusion");
    }

    private static RobotVision createRobotVision(
            AtomicBoolean hasTargets,
            AtomicReference<Pose2d> pose,
            double latencySeconds,
            String table) throws Exception {
        VisionCameraConfig config = VisionCameraConfig.create(table, CameraSoftware.PhotonVision)
                .sources(section -> section
                        .connectedSupplier(() -> true)
                        .hasTargetsSupplier(hasTargets::get)
                        .poseSupplier(pose::get)
                        .latencySupplier(() -> latencySeconds)
                        .visibleTargetsSupplier(() -> hasTargets.get() ? 1 : 0)
                        .averageDistanceSupplier(() -> 1.0))
                .simulation(section -> section.publishPoseTopic(true))
                .localization(section -> section
                        .useForLocalization(true)
                        .singleStdDevs(VecBuilder.fill(0.02, 0.02, 0.02))
                        .maxLatencySeconds(1.0));

        VisionCamera camera = new VisionCamera(config);

        RobotVision vision = RobotVision.RobotVisionConfig.defaults().create();
        Method registerMethod = RobotVision.class.getDeclaredMethod(
                "registerCamera",
                String.class,
                VisionCamera.class,
                Object.class);
        registerMethod.setAccessible(true);
        registerMethod.invoke(vision, table, camera, null);
        return vision;
    }

    private static RobotLocalization<DifferentialDriveWheelPositions> createLocalization(
            RobotLocalizationConfig localizationConfig,
            RobotSpeeds robotSpeeds,
            Imu imu,
            Supplier<DifferentialDriveWheelPositions> wheelPositions) {
        return new RobotLocalization<>(
                new DifferentialTestEstimatorFactory(0.62),
                localizationConfig,
                robotSpeeds,
                imu,
                wheelPositions);
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
