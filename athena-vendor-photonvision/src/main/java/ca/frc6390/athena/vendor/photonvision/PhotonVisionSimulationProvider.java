package ca.frc6390.athena.vendor.photonvision;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.runtime.VisionSimulation;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import ca.frc6390.athena.vision.runtime.VisionSimulationProvider;
import ca.frc6390.athena.vision.runtime.VisionSimulationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * PhotonVision-backed simulation provider for Athena camera declarations.
 */
public final class PhotonVisionSimulationProvider implements VisionSimulationProvider {
    @Override
    public boolean supports(List<CameraDevice> cameras) {
        return cameras != null && cameras.stream().anyMatch(Objects::nonNull);
    }

    @Override
    public VisionSimulation create(List<CameraDevice> cameras) {
        return create(cameras, VisionSimulationField.EMPTY);
    }

    @Override
    public VisionSimulation create(List<CameraDevice> cameras, VisionSimulationField field) {
        return new PhotonVisionSimulation(cameras, field);
    }

    static final class PhotonVisionSimulation implements VisionSimulation {
        private static final double DEFAULT_FOV_DEGREES = 70.0;
        private static final int DEFAULT_WIDTH = 640;
        private static final int DEFAULT_HEIGHT = 480;
        private static final double DEFAULT_FPS = 20.0;
        private static final double DEFAULT_LATENCY_MS = 35.0;
        private static final double DEFAULT_LATENCY_STD_DEV_MS = 5.0;
        private static final VisionSimulationTarget DEFAULT_TARGET =
                VisionSimulationTarget.aprilTag(1, 5.0, 0.0, 1.0, 0.0);

        private final VisionSystemSim visionSystem = createVisionSystem();
        private final Map<String, PhotonCameraSim> cameras = new LinkedHashMap<>();
        private final Map<String, CameraDevice> cameraDeclarations = new LinkedHashMap<>();
        private final Map<String, List<Measurement>> latestTargets = new LinkedHashMap<>();
        private final Map<String, List<Measurement>> latestPoses = new LinkedHashMap<>();
        private final VisionSimulationField field;

        PhotonVisionSimulation(List<CameraDevice> cameras, VisionSimulationField field) {
            VisionSimulationField safeField = field == null ? VisionSimulationField.EMPTY : field;
            this.field = safeField.targets().isEmpty()
                    ? VisionSimulationField.of(DEFAULT_TARGET)
                    : safeField;
            if (cameras != null) {
                cameras.stream()
                        .filter(Objects::nonNull)
                        .forEach(this::register);
            }
        }

        @Override
        public CameraDevice bind(CameraDevice camera) {
            if (camera == null || camera.hasBoundSignals()) {
                return camera;
            }
            String key = key(camera);
            return camera
                    .bindPose(() -> latestPoses.getOrDefault(key, List.of()))
                    .bindTargets(() -> latestTargets.getOrDefault(key, List.of()));
        }

        @Override
        public void update(PoseSnapshot robotPose) {
            update(robotPose, 0.0);
        }

        @Override
        public void update(PoseSnapshot robotPose, double timestampSeconds) {
            if (robotPose == null) {
                return;
            }
            if (visionSystem != null && Boolean.getBoolean("athena.photonvision.sim.native")) {
                visionSystem.update(new Pose2d(
                        robotPose.xMeters(),
                        robotPose.yMeters(),
                        Rotation2d.fromRadians(robotPose.headingRadians())));
            }
            cameraDeclarations.values().forEach(camera -> {
                List<Measurement> targets = simulatedTargets(robotPose, camera, field);
                latestTargets.put(key(camera), targets);
                latestPoses.put(key(camera), simulatedPose(robotPose, camera, targets, timestampSeconds));
            });
        }

        Map<String, PhotonCameraSim> cameras() {
            return Map.copyOf(cameras);
        }

        private void register(CameraDevice camera) {
            if (visionSystem != null && Boolean.getBoolean("athena.photonvision.sim.native")) {
                PhotonCameraSim sim = new PhotonCameraSim(new PhotonCamera(camera.name()), defaultProperties());
                visionSystem.addCamera(sim, transform(camera.mountPose()));
                cameras.put(key(camera), sim);
            }
            cameraDeclarations.put(key(camera), camera);
            latestTargets.put(key(camera), List.of());
            latestPoses.put(key(camera), List.of());
        }

        private static String key(CameraDevice camera) {
            return camera.kind().key() + ":" + camera.name();
        }

        static List<Measurement> simulatedTargets(PoseSnapshot robotPose, CameraDevice camera) {
            return simulatedTargets(robotPose, camera, VisionSimulationField.of(DEFAULT_TARGET));
        }

        static List<Measurement> simulatedTargets(
                PoseSnapshot robotPose,
                CameraDevice camera,
                VisionSimulationField field) {
            CameraMountPose mount = camera.mountPose() == null || !camera.mountPose().isFinite()
                    ? CameraMountPose.identity()
                    : camera.mountPose();
            VisionSimulationField safeField = field == null ? VisionSimulationField.EMPTY : field;
            return safeField.targets().stream()
                    .map(target -> simulatedTarget(robotPose, mount, target))
                    .filter(Objects::nonNull)
                    .map(target -> (Measurement) Measurements.target(
                            target.fiducialId(),
                            target.yawDegrees(),
                            target.pitchDegrees(),
                            target.distanceMeters(),
                            target.confidence()))
                    .toList();
        }

        private static PhotonVisionTarget simulatedTarget(
                PoseSnapshot robotPose,
                CameraMountPose mount,
                VisionSimulationTarget target) {
            double dx = target.pose().xMeters() - robotPose.xMeters();
            double dy = target.pose().yMeters() - robotPose.yMeters();
            double distance = Math.hypot(dx, dy);
            if (distance <= 1.0e-9) {
                return null;
            }
            double targetAngle = Math.atan2(dy, dx);
            double cameraYaw = robotPose.headingRadians() + Math.toRadians(mount.yawDegrees());
            double yawDegrees = Math.toDegrees(normalizeRadians(targetAngle - cameraYaw));
            if (Math.abs(yawDegrees) > DEFAULT_FOV_DEGREES * 0.5) {
                return null;
            }
            double pitchDegrees = Math.toDegrees(Math.atan2(target.heightMeters() - mount.zMeters(), distance))
                    - mount.pitchDegrees();
            return PhotonVisionTarget.aprilTag(
                    target.id(),
                    yawDegrees,
                    pitchDegrees,
                    distance,
                    Math.max(0.0, 1.0 - target.confidence()));
        }

        private static List<Measurement> simulatedPose(
                PoseSnapshot robotPose,
                CameraDevice camera,
                List<Measurement> targets,
                double timestampSeconds) {
            if (targets == null || targets.isEmpty()) {
                return List.of();
            }
            return List.of((Measurement) Measurements.pose(robotPose)
                    .timing(timestampSeconds, 0.0)
                    .visionMetadata(0.0, targets.size())
                    .source(camera));
        }

        private static double normalizeRadians(double radians) {
            double value = radians;
            while (value > Math.PI) {
                value -= Math.PI * 2.0;
            }
            while (value < -Math.PI) {
                value += Math.PI * 2.0;
            }
            return value;
        }

        private static SimCameraProperties defaultProperties() {
            SimCameraProperties properties = new SimCameraProperties();
            properties.setCalibration(DEFAULT_WIDTH, DEFAULT_HEIGHT, Rotation2d.fromDegrees(DEFAULT_FOV_DEGREES))
                    .setFPS(DEFAULT_FPS)
                    .setAvgLatencyMs(DEFAULT_LATENCY_MS)
                    .setLatencyStdDevMs(DEFAULT_LATENCY_STD_DEV_MS);
            return properties;
        }

        private static Transform3d transform(CameraMountPose pose) {
            CameraMountPose safePose = pose == null || !pose.isFinite() ? CameraMountPose.identity() : pose;
            return new Transform3d(
                    new Translation3d(safePose.xMeters(), safePose.yMeters(), safePose.zMeters()),
                    new Rotation3d(
                            Math.toRadians(safePose.rollDegrees()),
                            Math.toRadians(safePose.pitchDegrees()),
                            Math.toRadians(safePose.yawDegrees())));
        }

        private static VisionSystemSim createVisionSystem() {
            if (!Boolean.getBoolean("athena.photonvision.sim.native")) {
                return null;
            }
            try {
                return new VisionSystemSim("AthenaVisionSim");
            } catch (RuntimeException | LinkageError error) {
                return null;
            }
        }
    }
}
