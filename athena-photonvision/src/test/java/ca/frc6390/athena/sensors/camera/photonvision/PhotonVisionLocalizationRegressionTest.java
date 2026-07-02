package ca.frc6390.athena.sensors.camera.photonvision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.core.localization.PoseEstimatorFactory;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig.BackendConfig.VisionStrategy;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig.CameraRole;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;

final class PhotonVisionLocalizationRegressionTest {
    private static final String INTAKE_TABLE = "Intake";
    private static final double TRACKWIDTH_METERS = Units.inchesToMeters(22.5);
    private static final double WHEELBASE_METERS = Units.inchesToMeters(21.25);
    private static final Transform3d INTAKE_ROBOT_TO_CAMERA = new Transform3d(
            Units.inchesToMeters(3.0),
            Units.inchesToMeters(1.5),
            Units.inchesToMeters(15.35),
            new Rotation3d(0.0, Units.degreesToRadians(10.0), 0.0));

    @BeforeAll
    static void loadPhotonTargetingBaseLibrary() {
        String libraryPath = System.getProperty("java.library.path", "");
        for (String rawPath : libraryPath.split(java.io.File.pathSeparator)) {
            if (rawPath == null || rawPath.isBlank()) {
                continue;
            }
            Path candidate = Path.of(rawPath, System.mapLibraryName("photontargeting"));
            if (Files.isRegularFile(candidate)) {
                System.load(candidate.toAbsolutePath().toString());
                return;
            }
        }
    }

    @Test
    void rebuilt2026PhotonCameraAppliesVisionToFieldAndFieldVisionPoses() {
        PhotonVisionConfig cameraConfig = rebuilt2026IntakeCamera();
        RobotVision vision = RobotVision.RobotVisionConfig.defaults()
                .addCamera(cameraConfig)
                .create();

        VisionCamera intakeCamera = vision.cameras().camera(INTAKE_TABLE);
        assertTrue(intakeCamera.isUseForLocalization(), "Intake camera should be enabled for localization");
        assertTrue(intakeCamera.getRoles().contains(CameraRole.LOCALIZATION),
                "Intake camera should keep the Rebuilt LOCALIZATION role");

        RobotLocalization<SwerveModulePosition[]> localization = createRebuilt2026Localization();
        localization.setRobotVision(vision);
        localization.periodic();

        PhotonVision photon = photonCapability(vision);
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(cameraConfig.fieldLayout());
        Pose2d simulatedRobotPose = submitVisiblePhotonFrame(photon, cameraConfig, layout);

        Optional<VisionCamera.VisionMeasurement> measurement = vision.measurements().bestMeasurement();
        assertTrue(measurement.isPresent(), "PhotonVision should produce a localization measurement");
        Pose2d measuredPose = measurement.get().pose2d();
        assertTrue(measuredPose.getTranslation().getDistance(new Translation2d()) > 0.5,
                "Photon measurement should not be the default origin pose");

        localization.periodic();

        Pose2d fieldPose = localization.getPose2d("field");
        Pose2d fieldVisionPose = localization.getPose2d("fieldVision");
        assertVisionMovedPose("field", fieldPose, measuredPose, simulatedRobotPose);
        assertVisionMovedPose("fieldVision", fieldVisionPose, measuredPose, simulatedRobotPose);
    }

    private static PhotonVisionConfig rebuilt2026IntakeCamera() {
        return PhotonVisionConfig.table(INTAKE_TABLE)
                .addRole(CameraRole.LOCALIZATION)
                .withUseForLocalization(true)
                .withConfidence(1.0)
                .withTrustDistance(6.0)
                .withPoseStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
                .withCameraRobotSpace(INTAKE_ROBOT_TO_CAMERA);
    }

    private static RobotLocalization<SwerveModulePosition[]> createRebuilt2026Localization() {
        return new RobotLocalization<>(
                new SwerveTestEstimatorFactory(TRACKWIDTH_METERS, WHEELBASE_METERS),
                RobotLocalizationConfig.create()
                        .estimation(e -> e
                                .vision(0.3, 0.3, 9999.0)
                                .visionEnabled(true))
                        .poses(p -> p
                                .pose("field", pose -> pose
                                        .defaults()
                                        .backend(b -> b
                                                .visionStrategy(VisionStrategy.MULTI)
                                                .poseJumpMeters(100.0)
                                                .poseJumpAgreementMeters(100.0)
                                                .slip(s -> s.enabled(false))))
                                .pose("fieldVision", pose -> pose
                                        .vision()
                                        .backend(b -> b
                                                .visionStrategy(VisionStrategy.MULTI)
                                                .poseJumpMeters(100.0)
                                                .poseJumpAgreementMeters(100.0)
                                                .slip(s -> s.enabled(false))))),
                new RobotSpeeds(4.5, Math.PI),
                new TestImu(),
                PhotonVisionLocalizationRegressionTest::stationarySwerveModules);
    }

    private static PhotonVision photonCapability(RobotVision vision) {
        Object capability = vision.cameras()
                .capability(INTAKE_TABLE, VisionCameraCapability.PHOTON_VISION_CAMERA)
                .orElseThrow(() -> new AssertionError("Intake camera should expose PhotonVision capability"));
        return assertInstanceOf(PhotonVision.class, capability);
    }

    private static Pose2d submitVisiblePhotonFrame(
            PhotonVision photon,
            PhotonVisionConfig cameraConfig,
            AprilTagFieldLayout layout) {
        SimCameraProperties props = new SimCameraProperties()
                .setCalibration(
                        cameraConfig.simResolutionWidth(),
                        cameraConfig.simResolutionHeight(),
                        Rotation2d.fromDegrees(cameraConfig.simHorizontalFovDeg()))
                .setFPS(90.0)
                .setAvgLatencyMs(0.0)
                .setLatencyStdDevMs(0.0)
                .setRandomSeed(6390L);
        PhotonCameraSim sim = new PhotonCameraSim(photon, props, layout);
        sim.enableRawStream(false);
        sim.enableProcessedStream(false);
        sim.setMinTargetAreaPixels(1.0);

        List<VisionTargetSim> targets = layout.getTags().stream()
                .map(tag -> new VisionTargetSim(
                        layout.getTagPose(tag.ID).orElseThrow(),
                        TargetModel.kAprilTag36h11,
                        tag.ID))
                .toList();

        Pose2d multiTagFallbackPose = null;
        PhotonPipelineResult multiTagFallbackResult = null;
        Pose2d anyTargetFallbackPose = null;
        PhotonPipelineResult anyTargetFallbackResult = null;
        List<Pose2d> candidates = candidateRobotPoses(layout);
        assertFalse(candidates.isEmpty(), "default AprilTag layout should produce candidate robot poses");
        for (Pose2d robotPose : candidates) {
            Pose3d cameraPose = new Pose3d(robotPose).plus(cameraConfig.cameraRobotSpace());
            PhotonPipelineResult result = sim.process(0.0, cameraPose, targets);
            if (!result.hasTargets()) {
                continue;
            }
            if (result.getTargets().size() == 1) {
                sim.submitProcessedFrame(result);
                return robotPose;
            }
            if (result.getMultiTagResult().isPresent() && multiTagFallbackResult == null) {
                multiTagFallbackPose = robotPose;
                multiTagFallbackResult = result;
            }
            if (anyTargetFallbackResult == null) {
                anyTargetFallbackPose = robotPose;
                anyTargetFallbackResult = result;
            }
        }

        if (multiTagFallbackResult != null) {
            sim.submitProcessedFrame(multiTagFallbackResult);
            return multiTagFallbackPose;
        }
        if (anyTargetFallbackResult != null) {
            sim.submitProcessedFrame(anyTargetFallbackResult);
            return anyTargetFallbackPose;
        }
        throw new AssertionError("Intake PhotonVision simulation did not see any AprilTags");
    }

    private static List<Pose2d> candidateRobotPoses(AprilTagFieldLayout layout) {
        List<Pose2d> candidates = new ArrayList<>();
        double length = layout.getFieldLength();
        double width = layout.getFieldWidth();
        double[] headings = {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0, 300.0, 330.0};
        for (double x = 0.75; x <= length - 0.75; x += 0.75) {
            for (double y = 0.75; y <= width - 0.75; y += 0.75) {
                for (double heading : headings) {
                    candidates.add(new Pose2d(x, y, Rotation2d.fromDegrees(heading)));
                }
            }
        }
        return candidates;
    }

    private static void assertVisionMovedPose(
            String poseName,
            Pose2d fusedPose,
            Pose2d measuredPose,
            Pose2d simulatedRobotPose) {
        double fusedDistanceFromOrigin = fusedPose.getTranslation().getDistance(new Translation2d());
        double originDistanceToVision = measuredPose.getTranslation().getDistance(new Translation2d());
        double fusedDistanceToVision = fusedPose.getTranslation().getDistance(measuredPose.getTranslation());

        assertTrue(fusedDistanceFromOrigin > 0.05,
                poseName + " should move off the origin after a PhotonVision measurement");
        assertTrue(fusedDistanceToVision < originDistanceToVision,
                poseName + " should be closer to the Photon pose than an untouched origin estimate");
        assertTrue(fusedPose.getTranslation().getDistance(simulatedRobotPose.getTranslation()) < originDistanceToVision,
                poseName + " should move toward the simulated robot pose that produced the Photon frame");
    }

    private static SwerveModulePosition[] stationarySwerveModules() {
        return new SwerveModulePosition[] {
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d())
        };
    }

    private static final class SwerveTestEstimatorFactory implements PoseEstimatorFactory<SwerveModulePosition[]> {
        private final SwerveDriveKinematics kinematics;

        private SwerveTestEstimatorFactory(double trackWidthMeters, double wheelBaseMeters) {
            double halfTrack = trackWidthMeters / 2.0;
            double halfWheelBase = wheelBaseMeters / 2.0;
            this.kinematics = new SwerveDriveKinematics(
                    new Translation2d(halfWheelBase, halfTrack),
                    new Translation2d(halfWheelBase, -halfTrack),
                    new Translation2d(-halfWheelBase, halfTrack),
                    new Translation2d(-halfWheelBase, -halfTrack));
        }

        @Override
        public PoseEstimator<SwerveModulePosition[]> create2d(
                Pose2d startPose,
                Matrix<N3, N1> stateStdDevs,
                Matrix<N3, N1> visionStdDevs) {
            return new SwerveDrivePoseEstimator(
                    kinematics,
                    startPose.getRotation(),
                    stationarySwerveModules(),
                    startPose,
                    stateStdDevs,
                    visionStdDevs);
        }

        @Override
        public PoseEstimator3d<SwerveModulePosition[]> create3d(
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
        public void setInverted(boolean inverted) {}

        @Override
        public boolean isInverted() {
            return false;
        }

        @Override
        public double getXSpeedMetersPerSecond() {
            return 0.0;
        }

        @Override
        public double getYSpeedMetersPerSecond() {
            return 0.0;
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
    }
}
