package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.runtime.measurement.Measurement;
/**
 * Default camera declaration implementation.
 */
public final class GenericCameraDevice implements CameraDevice {
    private final CameraKind kind;
    private final String name;
    private final CameraMountPose mountPose;
    private final Supplier<? extends List<? extends Measurement>> poseMeasurements;
    private final Supplier<? extends List<? extends Measurement>> targetMeasurements;
    private final boolean poseBound;
    private final boolean targetBound;

    public GenericCameraDevice(CameraKind kind, String name) {
        this(kind, name, CameraMountPose.identity(), List::of, List::of, false, false);
    }

    private GenericCameraDevice(
            CameraKind kind,
            String name,
            CameraMountPose mountPose,
            Supplier<? extends List<? extends Measurement>> poseMeasurements,
            Supplier<? extends List<? extends Measurement>> targetMeasurements,
            boolean poseBound,
            boolean targetBound) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.name = name == null || name.isBlank() ? kind.key() : name;
        this.mountPose = mountPose == null ? CameraMountPose.identity() : mountPose;
        this.poseMeasurements = poseMeasurements == null ? List::of : poseMeasurements;
        this.targetMeasurements = targetMeasurements == null ? List::of : targetMeasurements;
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
        return new GenericCameraDevice(kind, name, pose, poseMeasurements, targetMeasurements, poseBound, targetBound);
    }

    @Override
    public GenericCameraDevice bindPose(Supplier<? extends List<? extends Measurement>> poseMeasurements) {
        return new GenericCameraDevice(kind, name, mountPose, poseMeasurements, targetMeasurements, true, targetBound);
    }

    @Override
    public GenericCameraDevice bindTargets(Supplier<? extends List<? extends Measurement>> targetMeasurements) {
        return new GenericCameraDevice(kind, name, mountPose, poseMeasurements, targetMeasurements, poseBound, true);
    }

    @Override
    public PoseSignal pose() {
        return new GenericPoseSignal(this, poseMeasurements);
    }

    @Override
    public TargetSignal targets() {
        return new GenericTargetSignal(this, targetMeasurements);
    }

    @Override
    public boolean hasBoundSignals() {
        return poseBound || targetBound;
    }
}
