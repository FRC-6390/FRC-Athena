package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.runtime.measurement.Measurement;
/**
 * Limelight camera declaration.
 */
public final class LimelightDevice implements CameraDevice {
    private final GenericCameraDevice camera;

    public LimelightDevice(String name) {
        this(new GenericCameraDevice(CameraKinds.LIMELIGHT, name));
    }

    private LimelightDevice(GenericCameraDevice camera) {
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
    public LimelightDevice mount(CameraMountPose pose) {
        return new LimelightDevice(camera.mount(pose));
    }

    @Override
    public LimelightDevice bindPose(Supplier<? extends List<? extends Measurement>> poseMeasurements) {
        return new LimelightDevice(camera.bindPose(poseMeasurements));
    }

    @Override
    public LimelightDevice bindTargets(Supplier<? extends List<? extends Measurement>> targetMeasurements) {
        return new LimelightDevice(camera.bindTargets(targetMeasurements));
    }

    @Override
    public LimelightPoseSignal pose() {
        return new LimelightPoseSignal(this, camera.pose());
    }

    @Override
    public TargetSignal targets() {
        return camera.targets();
    }

    /**
     * Returns this declaration as a generic camera device.
     *
     * @return camera device
     */
    public CameraDevice asCamera() {
        return this;
    }
}
