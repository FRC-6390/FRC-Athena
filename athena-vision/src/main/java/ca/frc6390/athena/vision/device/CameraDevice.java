package ca.frc6390.athena.vision.device;

import java.util.List;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.vision.signal.PoseSignal;
import ca.frc6390.athena.vision.signal.TargetSignal;
/**
 * Robot-facing camera declaration.
 */
public interface CameraDevice {
    /**
     * Returns the camera kind.
     *
     * @return camera kind
     */
    CameraKind kind();

    /**
     * Returns the camera device name or address.
     *
     * @return camera name
     */
    String name();

    /**
     * Returns the robot-relative mount pose.
     *
     * @return mount pose
     */
    CameraMountPose mountPose();

    /**
     * Returns a copy with a mount pose.
     *
     * @param pose mount pose
     * @return updated camera device
     */
    CameraDevice mount(CameraMountPose pose);

    /**
     * Returns a copy with a mount pose.
     *
     * @param xMeters x position
     * @param yMeters y position
     * @param zMeters z position
     * @param yawDegrees yaw
     * @param pitchDegrees pitch
     * @param rollDegrees roll
     * @return updated camera device
     */
    default CameraDevice mount(
            double xMeters,
            double yMeters,
            double zMeters,
            double yawDegrees,
            double pitchDegrees,
            double rollDegrees) {
        return mount(new CameraMountPose(xMeters, yMeters, zMeters, yawDegrees, pitchDegrees, rollDegrees));
    }

    /**
     * Returns a copy with a pose-measurement supplier.
     *
     * @param poseMeasurements pose measurement supplier
     * @return updated camera device
     */
    CameraDevice bindPose(Supplier<? extends List<? extends Measurement>> poseMeasurements);

    /**
     * Returns a copy with a target-measurement supplier.
     *
     * @param targetMeasurements target measurement supplier
     * @return updated camera device
     */
    CameraDevice bindTargets(Supplier<? extends List<? extends Measurement>> targetMeasurements);

    /**
     * Returns pose measurements from this camera.
     *
     * @return pose signal
     */
    PoseSignal pose();

    /**
     * Returns target measurements from this camera.
     *
     * @return target signal
     */
    TargetSignal targets();

    /**
     * Returns whether this declaration has explicit pose or target signal bindings.
     *
     * @return true when signals are externally supplied
     */
    default boolean hasBoundSignals() {
        return false;
    }
}
