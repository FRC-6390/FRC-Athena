package ca.frc6390.athena.vision.device;

import java.util.List;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.vision.signal.HeliosPoseSignal;
import ca.frc6390.athena.vision.signal.TargetSignal;
/**
 * HeliOS camera declaration.
 */
public final class HeliosDevice implements CameraDevice {
    private final GenericCameraDevice camera;

    public HeliosDevice(String address) {
        this(new GenericCameraDevice(CameraKinds.HELIOS, address));
    }

    private HeliosDevice(GenericCameraDevice camera) {
        this.camera = camera;
    }

    @Override
    public CameraKind kind() {
        return camera.kind();
    }

    @Override
    public FailurePolicy failurePolicy() { return camera.failurePolicy(); }

    @Override
    public HeliosDevice failurePolicy(FailurePolicy policy) {
        return new HeliosDevice(camera.failurePolicy(policy));
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
    public HeliosDevice mount(CameraMountPose pose) {
        return new HeliosDevice(camera.mount(pose));
    }

    @Override
    public HeliosDevice mount(Supplier<CameraMountPose> pose) {
        return new HeliosDevice(camera.mount(pose));
    }

    @Override
    public HeliosDevice bindPose(Supplier<? extends List<? extends Measurement>> poseMeasurements) {
        return new HeliosDevice(camera.bindPose(poseMeasurements));
    }

    @Override
    public HeliosDevice bindTargets(Supplier<? extends List<? extends Measurement>> targetMeasurements) {
        return new HeliosDevice(camera.bindTargets(targetMeasurements));
    }

    @Override
    public HeliosPoseSignal pose() {
        return new HeliosPoseSignal(this, camera.pose());
    }

    @Override
    public HeliosPoseSignal sourcePose() {
        return new HeliosPoseSignal(this, camera.sourcePose());
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
