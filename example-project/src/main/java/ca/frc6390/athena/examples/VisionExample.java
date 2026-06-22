package ca.frc6390.athena.examples;

import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.spec.CameraSpec;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;

/**
 * Vision example using generic camera declarations and observations.
 */
public final class VisionExample {
    /**
     * Front camera declaration.
     */
    public static final CameraSpec FRONT_CAMERA = Cameras.camera("front", camera -> camera
            .hardware(RobotHardware.FRONT_CAMERA)
            .mountPose(0.24, 0.0, 0.62, 0.0, -18.0, 0.0));

    private VisionExample() {
    }

    /**
     * Creates a sample frame with multiple AprilTag observations.
     *
     * @return sample vision frame
     */
    public static VisionFrame sampleFrame() {
        return VisionFrame.of(
                VisionObservation.tag(3, 8.5, -2.0, 4.1, 0.7),
                VisionObservation.tag(7, -3.2, -1.5, 2.4, 0.92));
    }
}
