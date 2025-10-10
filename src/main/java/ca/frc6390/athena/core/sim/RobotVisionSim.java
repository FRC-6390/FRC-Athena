package ca.frc6390.athena.core.sim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;

import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulation utilities for {@link RobotVision}. Wraps PhotonVision's {@link SimVisionSystem} and
 * {@link PhotonCameraSim} so simulated cameras publish the same NetworkTables data as hardware.
 */
public class RobotVisionSim {

    private static final double DEFAULT_FPS = 30.0;

    private final Map<String, CameraSimBundle> cameraSims = new HashMap<>();
    private final List<VisionTargetSim> targets;

    public RobotVisionSim(RobotVision vision, AprilTagFieldLayout layout) {
        this.targets = createTargets(layout);
        vision.getCameras().forEach((key, camera) ->
                vision.getPhotonVisionCamera(key).ifPresent(photonVision ->
                        registerPhotonVisionCamera(key, camera, photonVision, layout)));
    }

    private void registerPhotonVisionCamera(String key, LocalizationCamera camera,
                                            PhotonVision photonVision,
                                            AprilTagFieldLayout layout) {
        var config = photonVision.getConfig();
        SimCameraProperties props = SimCameraProperties.LL2_960_720().copy()
                .setCalibration(
                        config.simResolutionWidth(),
                        config.simResolutionHeight(),
                        Rotation2d.fromDegrees(diagonalFov(config.simHorizontalFovDeg(), config.simVerticalFovDeg())))
                .setFPS(DEFAULT_FPS);

        PhotonCameraSim cameraSim = new PhotonCameraSim(photonVision, props, layout);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        cameraSims.put(key, new CameraSimBundle(cameraSim, camera.getRobotToCameraTransform()));
    }

    /** Processes a single simulation frame using the provided robot pose. */
    public void update(Pose2d robotPose) {
        Pose3d robotPose3d = new Pose3d(robotPose);
        double timestamp = Timer.getFPGATimestamp();

        cameraSims.forEach((key, bundle) -> {
            Pose3d cameraPose = robotPose3d.transformBy(bundle.robotToCamera);
            var result = bundle.cameraSim.process(timestamp, cameraPose, targets);
            bundle.cameraSim.submitProcessedFrame(result);
            bundle.cameraSim.getVideoSimRaw().putFrame(bundle.cameraSim.getVideoSimFrameRaw());
        });
    }

    public Optional<PhotonCameraSim> getPhotonCameraSim(String key) {
        return Optional.ofNullable(cameraSims.get(key)).map(bundle -> bundle.cameraSim);
    }

    public Map<String, PhotonCameraSim> getPhotonCameraSims() {
        Map<String, PhotonCameraSim> map = new HashMap<>();
        cameraSims.forEach((k, v) -> map.put(k, v.cameraSim));
        return Collections.unmodifiableMap(map);
    }

    private static List<VisionTargetSim> createTargets(AprilTagFieldLayout layout) {
        List<VisionTargetSim> targets = new ArrayList<>();
        layout.getTags().forEach(tag -> targets.add(new VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11, tag.ID)));
        return targets;
    }

    private static double diagonalFov(double horizontal, double vertical) {
        return Math.hypot(horizontal, vertical);
    }

    private static class CameraSimBundle {
        private final PhotonCameraSim cameraSim;
        private final Transform3d robotToCamera;

        private CameraSimBundle(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
            this.cameraSim = cameraSim;
            this.robotToCamera = robotToCamera;
        }
    }
}
