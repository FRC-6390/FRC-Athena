package ca.frc6390.athena.vision.device;

import java.util.List;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.api.RecoveryPolicy;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.vision.signal.PhotonVisionPoseSignal;
import ca.frc6390.athena.vision.signal.TargetSignal;
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
    public FailurePolicy failurePolicy() { return camera.failurePolicy(); }

    @Override
    public PhotonVisionDevice failurePolicy(FailurePolicy policy) {
        return new PhotonVisionDevice(camera.failurePolicy(policy));
    }

    @Override
    public RecoveryPolicy recoveryPolicy() { return camera.recoveryPolicy(); }

    @Override
    public PhotonVisionDevice onRecovery(RecoveryPolicy policy) {
        return new PhotonVisionDevice(camera.onRecovery(policy));
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
    public PhotonVisionDevice mount(Supplier<CameraMountPose> pose) {
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
    public PhotonVisionPoseSignal sourcePose() {
        return new PhotonVisionPoseSignal(this, camera.sourcePose());
    }

    @Override
    public TargetSignal targets() {
        return camera.targets();
    }

    @Override
    public TargetSignal sourceTargets() {
        return camera.sourceTargets();
    }

    @Override
    public void bindRuntimeSignals(
            ca.frc6390.athena.vision.signal.PoseSignal poseSignal,
            TargetSignal targetSignal) {
        camera.bindRuntimeSignals(poseSignal, targetSignal);
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
