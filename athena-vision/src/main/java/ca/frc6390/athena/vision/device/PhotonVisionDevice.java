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
 *
 * <p>Copies made by {@code mount}, {@code bindPose} and similar methods share one runtime state
 * object, so options set through {@link #pose()} and the clock status published by a bound adapter
 * are visible from every copy of the declaration.</p>
 */
public final class PhotonVisionDevice implements CameraDevice, CameraClock {
    private final GenericCameraDevice camera;
    private final State state;

    public PhotonVisionDevice(String name) {
        this(new GenericCameraDevice(CameraKinds.PHOTONVISION, name), new State());
    }

    private PhotonVisionDevice(GenericCameraDevice camera, State state) {
        this.camera = camera;
        this.state = state;
    }

    @Override
    public CameraKind kind() {
        return camera.kind();
    }

    @Override
    public FailurePolicy failurePolicy() { return camera.failurePolicy(); }

    @Override
    public PhotonVisionDevice failurePolicy(FailurePolicy policy) {
        return new PhotonVisionDevice(camera.failurePolicy(policy), state);
    }

    @Override
    public RecoveryPolicy recoveryPolicy() { return camera.recoveryPolicy(); }

    @Override
    public PhotonVisionDevice onRecovery(RecoveryPolicy policy) {
        return new PhotonVisionDevice(camera.onRecovery(policy), state);
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
        return new PhotonVisionDevice(camera.mount(pose), state);
    }

    @Override
    public PhotonVisionDevice mount(Supplier<CameraMountPose> pose) {
        return new PhotonVisionDevice(camera.mount(pose), state);
    }

    @Override
    public PhotonVisionDevice bindPose(Supplier<? extends List<? extends Measurement>> poseMeasurements) {
        return new PhotonVisionDevice(camera.bindPose(poseMeasurements), state);
    }

    @Override
    public PhotonVisionDevice bindTargets(Supplier<? extends List<? extends Measurement>> targetMeasurements) {
        return new PhotonVisionDevice(camera.bindTargets(targetMeasurements), state);
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
     * Enables or disables the single-tag heading solve for this declaration.
     *
     * <p>When enabled, frames with exactly one tag are solved with PhotonLib's
     * {@code PNP_DISTANCE_TRIG_SOLVE} using the robot heading from
     * {@code PhotonVisionCameraAdapter.defaultRobotHeading}, falling back to the lowest-ambiguity
     * solve while no heading sample is available. Multi-tag frames are unaffected. Usually set
     * through {@code pose().singleTagUsesHeading()}.</p>
     *
     * @param enabled true to use the heading-constrained single-tag solve
     */
    public void configureSingleTagHeading(boolean enabled) {
        state.singleTagUsesHeading = enabled;
    }

    /**
     * Returns whether the heading-constrained single-tag solve is enabled.
     *
     * @return true when enabled
     */
    public boolean singleTagHeadingEnabled() {
        return state.singleTagUsesHeading;
    }

    /**
     * Connects this declaration to the clock status of its bound adapter. Called by the adapter when it binds.
     *
     * @param status supplier of the latest clock status; null restores {@link CameraClockStatus#unknown()}
     */
    public void bindClockStatus(Supplier<CameraClockStatus> status) {
        state.clockStatus = status == null ? CameraClockStatus::unknown : status;
    }

    /**
     * Returns the camera clock status as of the most recent adapter refresh.
     *
     * <p>Returns {@link CameraClockStatus#unknown()} until a PhotonVision adapter has bound this
     * declaration and refreshed at least once.</p>
     *
     * @return clock status; never null
     */
    @Override
    public CameraClockStatus clockStatus() {
        CameraClockStatus status = state.clockStatus.get();
        return status == null ? CameraClockStatus.unknown() : status;
    }

    /**
     * Returns this declaration as a generic camera device.
     *
     * @return camera device
     */
    public CameraDevice asCamera() {
        return this;
    }

    /** Runtime options shared by all copies of one declaration. */
    private static final class State {
        private volatile boolean singleTagUsesHeading;
        private volatile Supplier<CameraClockStatus> clockStatus = CameraClockStatus::unknown;
    }
}
