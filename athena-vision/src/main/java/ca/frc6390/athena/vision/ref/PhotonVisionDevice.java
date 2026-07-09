package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.runtime.measurement.Measurement;
/**
 * PhotonVision camera declaration.
 */
public final class PhotonVisionDevice implements CameraDevice {
    private final GenericCameraDevice camera;

    public PhotonVisionDevice(String name) {
        this(new GenericCameraDevice(CameraKinds.PHOTONVISION, name));
    }

    private PhotonVisionDevice(GenericCameraDevice camera) {
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
    public PhotonVisionDevice mount(CameraMountPose pose) {
        return new PhotonVisionDevice(camera.mount(pose));
    }

    @Override
    public PhotonVisionDevice bindPose(Supplier<? extends List<? extends Measurement>> poseMeasurements) {
        return new PhotonVisionDevice(camera.bindPose(poseMeasurements));
    }

    @Override
    public PhotonVisionDevice bindTargets(Supplier<? extends List<? extends Measurement>> targetMeasurements) {
        return new PhotonVisionDevice(camera.bindTargets(targetMeasurements));
    }

    @Override
    public PhotonVisionPoseSignal pose() {
        return new PhotonVisionPoseSignal(this, camera.pose());
    }

    @Override
    public TargetSignal targets() {
        return camera.targets();
    }

    @Override
    public boolean hasBoundSignals() {
        return camera.hasBoundSignals();
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
