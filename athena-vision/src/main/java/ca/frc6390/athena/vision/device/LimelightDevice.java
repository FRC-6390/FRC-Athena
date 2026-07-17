package ca.frc6390.athena.vision.device;

import java.util.List;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.vision.signal.LimelightPoseSignal;
import ca.frc6390.athena.vision.signal.TargetSignal;
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
    public FailurePolicy failurePolicy() { return camera.failurePolicy(); }

    @Override
    public LimelightDevice failurePolicy(FailurePolicy policy) {
        return new LimelightDevice(camera.failurePolicy(policy));
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
    public LimelightDevice mount(Supplier<CameraMountPose> pose) {
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
    public LimelightPoseSignal sourcePose() {
        return new LimelightPoseSignal(this, camera.sourcePose());
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
