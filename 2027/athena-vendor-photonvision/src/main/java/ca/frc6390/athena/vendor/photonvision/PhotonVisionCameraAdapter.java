package ca.frc6390.athena.vendor.photonvision;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.vision.spec.CameraSpec;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision camera adapter backed by PhotonLib.
 */
public final class PhotonVisionCameraAdapter implements AutoCloseable {
    private final CameraSpec spec;
    private final PhotonClient client;

    /**
     * Creates an adapter for support checks and static target conversion only.
     */
    public PhotonVisionCameraAdapter() {
        spec = null;
        client = null;
    }

    /**
     * Creates an adapter using a real PhotonLib camera.
     *
     * @param spec camera spec
     */
    public PhotonVisionCameraAdapter(CameraSpec spec) {
        this(spec, new PhotonCameraClient(new PhotonCamera(Objects.requireNonNull(spec, "spec").cameraName())));
    }

    PhotonVisionCameraAdapter(CameraSpec spec, PhotonClient client) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.client = Objects.requireNonNull(client, "client");
    }

    /**
     * Returns true when a camera kind is handled by this adapter.
     *
     * @param kind camera kind
     * @return true if supported
     */
    public boolean supports(CameraKind kind) {
        return kind == AthenaCamera.PHOTONVISION || kind.key().equals("photonvision:camera");
    }

    /**
     * Returns true when a camera spec is handled by this adapter.
     *
     * @param spec camera spec
     * @return true if supported
     */
    public boolean supports(CameraSpec spec) {
        return spec != null && supports(spec.kind());
    }

    /**
     * Returns the latest unread PhotonVision frame from the configured camera.
     *
     * @return latest vision frame
     */
    public VisionFrame latestFrame() {
        if (client == null) {
            throw new IllegalStateException("PhotonVision camera adapter was created without a camera spec.");
        }
        List<PhotonPipelineResult> results = client.unreadResults();
        if (results.isEmpty()) {
            return VisionFrame.noTarget();
        }
        return frameFromResult(results.get(results.size() - 1));
    }

    /**
     * Returns the camera spec used by this adapter.
     *
     * @return camera spec
     */
    public CameraSpec spec() {
        return spec;
    }

    /**
     * Converts PhotonVision-shaped targets into Athena's generic frame model.
     *
     * @param targets target values
     * @return vision frame
     */
    public VisionFrame frame(List<PhotonVisionTarget> targets) {
        return frameFromTargets(targets);
    }

    /**
     * Converts PhotonVision-shaped targets into Athena's generic frame model.
     *
     * @param targets target values
     * @return vision frame
     */
    public static VisionFrame frameFromTargets(List<PhotonVisionTarget> targets) {
        if (targets == null || targets.isEmpty()) {
            return VisionFrame.noTarget();
        }
        return new VisionFrame(targets.stream()
                .map(PhotonVisionTarget::toObservation)
                .toList());
    }

    /**
     * Converts a real PhotonLib pipeline result into Athena's generic frame model.
     *
     * @param result PhotonLib pipeline result
     * @return vision frame
     */
    public static VisionFrame frameFromResult(PhotonPipelineResult result) {
        if (result == null || !result.hasTargets()) {
            return VisionFrame.noTarget();
        }
        return new VisionFrame(result.getTargets().stream()
                .map(PhotonVisionCameraAdapter::observationFromTarget)
                .toList());
    }

    /**
     * Converts a real PhotonLib tracked target into Athena's observation model.
     *
     * @param target PhotonLib target
     * @return vision observation
     */
    public static VisionObservation observationFromTarget(PhotonTrackedTarget target) {
        Objects.requireNonNull(target, "target");
        var transform = target.getBestCameraToTarget();
        var translation = transform.getTranslation();
        double distance = translation.getNorm();
        double confidence = Double.isFinite(target.getPoseAmbiguity())
                ? Math.max(0.0, 1.0 - target.getPoseAmbiguity())
                : 0.0;
        return new VisionObservation(
                target.getFiducialId(),
                target.getYaw(),
                target.getPitch(),
                distance,
                translation.getX(),
                translation.getY(),
                confidence);
    }

    @Override
    public void close() {
        if (client != null) {
            client.close();
        }
    }

    interface PhotonClient extends AutoCloseable {
        List<PhotonPipelineResult> unreadResults();

        @Override
        void close();
    }

    private static final class PhotonCameraClient implements PhotonClient {
        private final PhotonCamera camera;

        private PhotonCameraClient(PhotonCamera camera) {
            this.camera = Objects.requireNonNull(camera, "camera");
        }

        @Override
        public List<PhotonPipelineResult> unreadResults() {
            return camera.getAllUnreadResults();
        }

        @Override
        public void close() {
            camera.close();
        }
    }
}
