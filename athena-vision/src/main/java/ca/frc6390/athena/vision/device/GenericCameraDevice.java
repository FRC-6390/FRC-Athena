package ca.frc6390.athena.vision.device;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
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
    private final CameraMountPose mountPose;
    private final SignalBinding signals;
    private final boolean poseBound;
    private final boolean targetBound;

    public GenericCameraDevice(CameraKind kind, String name) {
        this(kind, name, CameraMountPose.identity(), new SignalBinding(), false, false);
    }

    private GenericCameraDevice(
            CameraKind kind,
            String name,
            CameraMountPose mountPose,
            SignalBinding signals,
            boolean poseBound,
            boolean targetBound) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.name = name == null || name.isBlank() ? kind.key() : name;
        this.mountPose = mountPose == null ? CameraMountPose.identity() : mountPose;
        this.signals = Objects.requireNonNull(signals, "signals");
        this.poseBound = poseBound;
        this.targetBound = targetBound;
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
        return mountPose;
    }

    @Override
    public GenericCameraDevice mount(CameraMountPose pose) {
        return new GenericCameraDevice(kind, name, pose, signals, poseBound, targetBound);
    }

    @Override
    public GenericCameraDevice bindPose(Supplier<? extends List<? extends Measurement>> poseMeasurements) {
        signals.bindPose(poseMeasurements);
        return new GenericCameraDevice(kind, name, mountPose, signals, true, targetBound);
    }

    @Override
    public GenericCameraDevice bindTargets(Supplier<? extends List<? extends Measurement>> targetMeasurements) {
        signals.bindTargets(targetMeasurements);
        return new GenericCameraDevice(kind, name, mountPose, signals, poseBound, true);
    }

    @Override
    public PoseSignal pose() {
        return new GenericPoseSignal(this, signals::poseMeasurements);
    }

    @Override
    public TargetSignal targets() {
        return new GenericTargetSignal(this, signals::targetMeasurements);
    }

    @Override
    public boolean hasBoundSignals() {
        return poseBound || targetBound;
    }

    private static final class SignalBinding {
        private volatile Supplier<? extends List<? extends Measurement>> poseMeasurements = List::of;
        private volatile Supplier<? extends List<? extends Measurement>> targetMeasurements = List::of;

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
}
