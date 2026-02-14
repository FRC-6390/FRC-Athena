package ca.frc6390.athena.sensors.camera.photonvision.sim;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.core.sim.RobotVisionSim;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVision;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotonVisionRobotVisionSim extends RobotVisionSim {
    private static final double DEFAULT_LATENCY_MS = 35.0;
    private static final int DEFAULT_WIDTH = 640;
    private static final int DEFAULT_HEIGHT = 480;
    private static final double DEFAULT_FOV_DEG = 70.0;

    private final VisionSystemSim visionSystem;
    private final Map<String, PhotonCameraSim> cameraSims = new HashMap<>();
    private final Map<String, LimelightSimState> limelightSims = new HashMap<>();

    public PhotonVisionRobotVisionSim(RobotVision vision, AprilTagFieldLayout layout) {
        super(vision, layout);
        this.visionSystem = new VisionSystemSim("AthenaVisionSim");
        if (layout != null) {
            visionSystem.addAprilTags(layout);
        }
        registerCameras();
    }

    @Override
    public void update(Pose2d robotPose) {
        visionSystem.update(robotPose);
        for (LimelightSimState sim : limelightSims.values()) {
            sim.update(robotPose);
        }
    }

    private void registerCameras() {
        for (VisionCamera camera : vision.cameras().all().values()) {
            if (camera.supports(VisionCameraCapability.PHOTON_VISION_CAMERA)) {
                registerPhotonVision(camera);
            } else if (camera.supports(VisionCameraCapability.LIMELIGHT_CAMERA)) {
                registerLimelight(camera);
            }
        }
    }

    private void registerPhotonVision(VisionCamera camera) {
        PhotonVision photon = camera.capability(VisionCameraCapability.PHOTON_VISION_CAMERA)
                .filter(PhotonVision.class::isInstance)
                .map(PhotonVision.class::cast)
                .orElse(null);
        if (photon == null) {
            return;
        }
        SimCameraProperties props = buildPhotonVisionProperties(photon.getConfig(), camera.getConfig().getDisplayHorizontalFov());
        PhotonCameraSim sim = new PhotonCameraSim(photon, props);
        addCamera(sim, camera.getConfig().getRobotToCameraTransform());
    }

    private void registerLimelight(VisionCamera camera) {
        String name = camera.getConfig().getTable();
        SimCameraProperties props = buildLimelightProperties(camera.getConfig().getDisplayHorizontalFov());
        PhotonCamera photonCamera = new PhotonCamera(name + "-sim");
        PhotonCameraSim sim = new PhotonCameraSim(photonCamera, props);
        addCamera(sim, camera.getConfig().getRobotToCameraTransform());
        limelightSims.put(name, new LimelightSimState(name, photonCamera));
    }

    private void addCamera(PhotonCameraSim sim, Transform3d robotToCamera) {
        visionSystem.addCamera(sim, robotToCamera != null ? robotToCamera : new Transform3d());
        cameraSims.put(sim.getCamera().getName(), sim);
    }

    private SimCameraProperties buildPhotonVisionProperties(PhotonVisionConfig config, double fallbackFovDeg) {
        SimCameraProperties props = new SimCameraProperties();
        int width = config.simResolutionWidth() > 0 ? config.simResolutionWidth() : DEFAULT_WIDTH;
        int height = config.simResolutionHeight() > 0 ? config.simResolutionHeight() : DEFAULT_HEIGHT;
        double fov = Double.isFinite(config.simHorizontalFovDeg()) ? config.simHorizontalFovDeg() : fallbackFovDeg;
        if (!Double.isFinite(fov) || fov <= 0.0) {
            fov = DEFAULT_FOV_DEG;
        }
        props.setCalibration(width, height, Rotation2d.fromDegrees(fov))
                .setFPS(20.0)
                .setAvgLatencyMs(DEFAULT_LATENCY_MS)
                .setLatencyStdDevMs(5.0);
        return props;
    }

    private SimCameraProperties buildLimelightProperties(double fallbackFovDeg) {
        SimCameraProperties props;
        if (Double.isFinite(fallbackFovDeg) && fallbackFovDeg > 0.0) {
            props = new SimCameraProperties()
                    .setCalibration(DEFAULT_WIDTH, DEFAULT_HEIGHT, Rotation2d.fromDegrees(fallbackFovDeg));
        } else {
            props = SimCameraProperties.LL2_960_720().copy();
        }
        props.setFPS(20.0)
                .setAvgLatencyMs(DEFAULT_LATENCY_MS)
                .setLatencyStdDevMs(5.0);
        return props;
    }

    private static final class LimelightSimState {
        private final PhotonCamera camera;
        private final NetworkTable table;
        private final NetworkTableEntry tv;
        private final NetworkTableEntry tx;
        private final NetworkTableEntry ty;
        private final NetworkTableEntry ta;
        private final NetworkTableEntry tid;
        private final NetworkTableEntry tl;
        private final List<NetworkTableEntry> botposeEntries;

        private LimelightSimState(String tableName, PhotonCamera camera) {
            this.camera = camera;
            this.table = NetworkTableInstance.getDefault().getTable(tableName);
            this.tv = table.getEntry("tv");
            this.tx = table.getEntry("tx");
            this.ty = table.getEntry("ty");
            this.ta = table.getEntry("ta");
            this.tid = table.getEntry("tid");
            this.tl = table.getEntry("tl");
            this.botposeEntries = List.of(
                    table.getEntry("botpose"),
                    table.getEntry("botpose_wpired"),
                    table.getEntry("botpose_wpiblue"),
                    table.getEntry("botpose_orb"),
                    table.getEntry("botpose_orb_wpired"),
                    table.getEntry("botpose_orb_wpiblue"));
        }

        private void update(Pose2d robotPose) {
            PhotonPipelineResult result = getLatestResult();
            boolean hasTargets = result != null && result.hasTargets();
            tv.setNumber(hasTargets ? 1 : 0);
            tx.setNumber(Double.NaN);
            ty.setNumber(Double.NaN);
            ta.setNumber(0.0);
            tid.setNumber(-1);
            if (result != null) {
                tl.setNumber(getLatencyMillis(result));
            }

            int tagCount = 0;
            double avgDistance = Double.NaN;
            double avgArea = 0.0;
            if (hasTargets) {
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                if (bestTarget != null) {
                    tx.setNumber(bestTarget.getYaw());
                    ty.setNumber(bestTarget.getPitch());
                    ta.setNumber(bestTarget.getArea());
                    tid.setNumber(bestTarget.getFiducialId());
                    if (bestTarget.getBestCameraToTarget() != null) {
                        avgDistance = bestTarget.getBestCameraToTarget().getTranslation().getNorm();
                    }
                }
                List<PhotonTrackedTarget> targets = result.getTargets();
                tagCount = targets != null ? targets.size() : 0;
                if (targets != null && !targets.isEmpty()) {
                    avgArea = targets.stream().mapToDouble(PhotonTrackedTarget::getArea).average().orElse(0.0);
                }
            }

            double latencyMs = result != null ? getLatencyMillis(result) : DEFAULT_LATENCY_MS;
            Double[] pose = new Double[] {
                    robotPose.getX(),
                    robotPose.getY(),
                    0.0,
                    0.0,
                    0.0,
                    robotPose.getRotation().getDegrees(),
                    latencyMs,
                    (double) tagCount,
                    Double.isFinite(avgDistance) ? avgDistance : Double.MAX_VALUE,
                    avgArea
            };
            for (NetworkTableEntry entry : botposeEntries) {
                entry.setDoubleArray(pose);
            }
        }

        private PhotonPipelineResult getLatestResult() {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            if (results == null || results.isEmpty()) {
                return null;
            }
            return results.get(results.size() - 1);
        }

        private double getLatencyMillis(PhotonPipelineResult result) {
            if (result == null || result.metadata == null) {
                return DEFAULT_LATENCY_MS;
            }
            return result.metadata.getLatencyMillis();
        }
    }
}
