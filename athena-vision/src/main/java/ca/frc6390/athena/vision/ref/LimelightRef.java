package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.runtime.measurement.PoseMeasurement;
import ca.frc6390.athena.vision.spec.CameraMountPose;
import ca.frc6390.athena.vision.spec.VisionFrame;

/**
 * Limelight-specific camera ref.
 */
public final class LimelightRef implements CameraRef {
    private final GenericCameraRef camera;

    LimelightRef(String name) {
        this(new GenericCameraRef(AthenaCamera.LIMELIGHT, name));
    }

    private LimelightRef(GenericCameraRef camera) {
        this.camera = camera;
    }

    @Override
    public CameraKind kind() {
        return camera.kind();
    }

    @Override
    public String name() {
        return camera.name();
    }

    @Override
    public CameraMountPose mountPose() {
        return camera.mountPose();
    }

    @Override
    public VisionFrame frame() {
        return camera.frame();
    }

    @Override
    public LimelightRef mount(CameraMountPose pose) {
        return new LimelightRef(camera.mount(pose));
    }

    /**
     * Returns a copy with a mount pose.
     *
     * @param xMeters x position
     * @param yMeters y position
     * @param zMeters z position
     * @param yawDegrees yaw
     * @param pitchDegrees pitch
     * @param rollDegrees roll
     * @return updated ref
     */
    public LimelightRef mount(
            double xMeters,
            double yMeters,
            double zMeters,
            double yawDegrees,
            double pitchDegrees,
            double rollDegrees) {
        return mount(new CameraMountPose(xMeters, yMeters, zMeters, yawDegrees, pitchDegrees, rollDegrees));
    }

    @Override
    public LimelightRef bindFrame(Supplier<VisionFrame> frame) {
        return new LimelightRef(camera.bindFrame(frame));
    }

    @Override
    public LimelightRef bindPose(Supplier<? extends List<? extends PoseMeasurement>> poseMeasurements) {
        return new LimelightRef(camera.bindPose(poseMeasurements));
    }

    @Override
    public LimelightPoseRef pose() {
        return new LimelightPoseRef(this, camera.pose());
    }

    @Override
    public CameraTargetRef targets() {
        return camera.targets();
    }

    /**
     * Returns this camera as a generic camera ref.
     *
     * @return camera ref
     */
    public CameraRef asCamera() {
        return this;
    }

}
