package ca.frc6390.athena.vision.device;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.vision.signal.GenericPoseSignal;
import ca.frc6390.athena.vision.signal.GenericTargetSignal;
import ca.frc6390.athena.vision.signal.PoseSignal;
import ca.frc6390.athena.vision.signal.TargetSignal;
/**
 * Default camera declaration implementation.
 */
public final class GenericCameraDevice implements CameraDevice {
    private final CameraKind kind;
    private final String name;
    private final Supplier<CameraMountPose> mountPose;
    private final SignalBinding signals;
    private final boolean poseBound;
    private final boolean targetBound;
    private final FailurePolicy failurePolicy;

    public GenericCameraDevice(CameraKind kind, String name) {
        this(kind, name, CameraMountPose::identity, new SignalBinding(), false, false,
                FailurePolicy.DISABLE_MECHANISM);
    }

    private GenericCameraDevice(
            CameraKind kind,
            String name,
            Supplier<CameraMountPose> mountPose,
            SignalBinding signals,
            boolean poseBound,
            boolean targetBound,
            FailurePolicy failurePolicy) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.name = name == null || name.isBlank() ? kind.key() : name;
        this.mountPose = mountPose == null ? CameraMountPose::identity : mountPose;
        this.signals = Objects.requireNonNull(signals, "signals");
        this.poseBound = poseBound;
        this.targetBound = targetBound;
        this.failurePolicy = failurePolicy == null ? FailurePolicy.DISABLE_MECHANISM : failurePolicy;
    }

    @Override
    public CameraKind kind() {
        return kind;
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public CameraMountPose mountPose() {
        CameraMountPose pose = mountPose.get();
        return pose == null ? CameraMountPose.identity() : pose;
    }

    @Override
    public GenericCameraDevice mount(CameraMountPose pose) {
        CameraMountPose safe = pose == null ? CameraMountPose.identity() : pose;
        return new GenericCameraDevice(kind, name, () -> safe, signals, poseBound, targetBound, failurePolicy);
    }

    @Override
    public GenericCameraDevice mount(Supplier<CameraMountPose> pose) {
        return new GenericCameraDevice(kind, name, pose, signals, poseBound, targetBound, failurePolicy);
    }

    @Override
    public GenericCameraDevice bindPose(Supplier<? extends List<? extends Measurement>> poseMeasurements) {
        signals.bindPose(poseMeasurements);
        return new GenericCameraDevice(kind, name, mountPose, signals, true, targetBound, failurePolicy);
    }

    @Override
    public GenericCameraDevice bindTargets(Supplier<? extends List<? extends Measurement>> targetMeasurements) {
        signals.bindTargets(targetMeasurements);
        return new GenericCameraDevice(kind, name, mountPose, signals, poseBound, true, failurePolicy);
    }

    @Override
    public FailurePolicy failurePolicy() {
        return failurePolicy;
    }

    @Override
    public GenericCameraDevice failurePolicy(FailurePolicy policy) {
        return new GenericCameraDevice(kind, name, mountPose, signals, poseBound, targetBound,
                Objects.requireNonNull(policy, "policy"));
    }

    @Override
    public PoseSignal pose() {
        return new BoundPoseSignal(this, signals);
    }

    @Override
    public PoseSignal sourcePose() {
        return new GenericPoseSignal(this, signals::poseMeasurements);
    }

    @Override
    public TargetSignal targets() {
        return new BoundTargetSignal(this, signals);
    }

    @Override
    public TargetSignal sourceTargets() {
        return new GenericTargetSignal(this, signals::targetMeasurements);
    }

    @Override
    public void bindRuntimeSignals(PoseSignal poseSignal, TargetSignal targetSignal) {
        signals.runtimePoseSignal = Objects.requireNonNull(poseSignal, "poseSignal");
        signals.runtimeTargetSignal = Objects.requireNonNull(targetSignal, "targetSignal");
    }

    @Override
    public boolean hasBoundSignals() {
        return poseBound || targetBound;
    }

    private static final class SignalBinding {
        private volatile Supplier<? extends List<? extends Measurement>> poseMeasurements = List::of;
        private volatile Supplier<? extends List<? extends Measurement>> targetMeasurements = List::of;
        private volatile PoseSignal runtimePoseSignal;
        private volatile TargetSignal runtimeTargetSignal;

        private void bindPose(Supplier<? extends List<? extends Measurement>> measurements) {
            poseMeasurements = measurements == null ? List::of : measurements;
        }

        private void bindTargets(Supplier<? extends List<? extends Measurement>> measurements) {
            targetMeasurements = measurements == null ? List::of : measurements;
        }

        private List<? extends Measurement> poseMeasurements() {
            List<? extends Measurement> values = poseMeasurements.get();
            return values == null ? List.of() : values;
        }

        private List<? extends Measurement> targetMeasurements() {
            List<? extends Measurement> values = targetMeasurements.get();
            return values == null ? List.of() : values;
        }
    }

    /** Resolves the runtime cache lazily so declarations captured before startup stay live. */
    private record BoundPoseSignal(GenericCameraDevice camera, SignalBinding signals) implements PoseSignal {
        @Override
        public List<Measurement> measurements() {
            PoseSignal runtime = signals.runtimePoseSignal;
            return runtime == null ? List.copyOf(signals.poseMeasurements()) : runtime.measurements();
        }

        @Override
        public Map<String, Object> metadata() {
            PoseSignal runtime = signals.runtimePoseSignal;
            return runtime == null ? Map.of() : runtime.metadata();
        }
    }

    /** Resolves the runtime cache lazily so declarations captured before startup stay live. */
    private record BoundTargetSignal(GenericCameraDevice camera, SignalBinding signals) implements TargetSignal {
        @Override
        public List<Measurement> measurements() {
            TargetSignal runtime = signals.runtimeTargetSignal;
            return runtime == null ? List.copyOf(signals.targetMeasurements()) : runtime.measurements();
        }

        @Override
        public Map<String, Object> metadata() {
            TargetSignal runtime = signals.runtimeTargetSignal;
            return runtime == null ? Map.of() : runtime.metadata();
        }
    }
}
