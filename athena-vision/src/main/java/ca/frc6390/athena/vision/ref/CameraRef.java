package ca.frc6390.athena.vision.ref;

import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.runtime.measurement.PoseMeasurement;
import ca.frc6390.athena.vision.spec.CameraMountPose;
import ca.frc6390.athena.vision.spec.VisionFrame;

/**
 * Generic camera reference used by vendor-specific camera refs and localization.
 */
public interface CameraRef {
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
     * Returns the latest camera frame.
     *
     * @return frame
     */
    VisionFrame frame();

    /**
     * Returns a copy with a mount pose.
     *
     * @param pose mount pose
     * @return updated camera ref
     */
    CameraRef mount(CameraMountPose pose);

    /**
     * Returns a copy with a mount pose.
     *
     * @param xMeters x position
     * @param yMeters y position
     * @param zMeters z position
     * @param yawDegrees yaw
     * @param pitchDegrees pitch
     * @param rollDegrees roll
     * @return updated camera ref
     */
    default CameraRef mount(
            double xMeters,
            double yMeters,
            double zMeters,
            double yawDegrees,
            double pitchDegrees,
            double rollDegrees) {
        return mount(new CameraMountPose(xMeters, yMeters, zMeters, yawDegrees, pitchDegrees, rollDegrees));
    }

    /**
     * Returns a copy with a frame supplier.
     *
     * @param frame frame supplier
     * @return updated camera ref
     */
    CameraRef bindFrame(Supplier<VisionFrame> frame);

    /**
     * Returns a copy with a pose-measurement supplier.
     *
     * @param poseMeasurements pose measurement supplier
     * @return updated camera ref
     */
    CameraRef bindPose(Supplier<? extends java.util.List<? extends PoseMeasurement>> poseMeasurements);

    /**
     * Returns pose measurements from this camera.
     *
     * @return pose measurement ref
     */
    CameraPoseRef pose();

    /**
     * Returns target measurements from this camera.
     *
     * @return target measurement ref
     */
    CameraTargetRef targets();
}
