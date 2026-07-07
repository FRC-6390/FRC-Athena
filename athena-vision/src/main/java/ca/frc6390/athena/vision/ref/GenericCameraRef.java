package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.runtime.measurement.PoseMeasurement;
import ca.frc6390.athena.vision.spec.CameraMountPose;
import ca.frc6390.athena.vision.spec.VisionFrame;

/**
 * Default camera ref implementation.
 */
public final class GenericCameraRef implements CameraRef {
    private final CameraKind kind;
    private final String name;
    private final CameraMountPose mountPose;
    private final Supplier<VisionFrame> frame;
    private final Supplier<? extends List<? extends PoseMeasurement>> poseMeasurements;

    public GenericCameraRef(CameraKind kind, String name) {
        this(kind, name, CameraMountPose.identity(), VisionFrame::noTarget, List::of);
    }

    private GenericCameraRef(
            CameraKind kind,
            String name,
            CameraMountPose mountPose,
            Supplier<VisionFrame> frame,
            Supplier<? extends List<? extends PoseMeasurement>> poseMeasurements) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.name = name == null || name.isBlank() ? kind.key() : name;
        this.mountPose = mountPose == null ? CameraMountPose.identity() : mountPose;
        this.frame = frame == null ? VisionFrame::noTarget : frame;
        this.poseMeasurements = poseMeasurements == null ? List::of : poseMeasurements;
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
    public VisionFrame frame() {
        VisionFrame value = frame.get();
        return value == null ? VisionFrame.noTarget() : value;
    }

    @Override
    public GenericCameraRef mount(CameraMountPose pose) {
        return new GenericCameraRef(kind, name, pose, frame, poseMeasurements);
    }

    @Override
    public GenericCameraRef bindFrame(Supplier<VisionFrame> frame) {
        return new GenericCameraRef(kind, name, mountPose, frame, poseMeasurements);
    }

    @Override
    public GenericCameraRef bindPose(Supplier<? extends List<? extends PoseMeasurement>> poseMeasurements) {
        return new GenericCameraRef(kind, name, mountPose, frame, poseMeasurements);
    }

    @Override
    public CameraPoseRef pose() {
        return new GenericCameraPoseRef(this, poseMeasurements);
    }

    @Override
    public CameraTargetRef targets() {
        return new GenericCameraTargetRef(this);
    }
}
