package ca.frc6390.athena.sim.world;

import ca.frc6390.athena.vision.spec.CameraSpec;
import ca.frc6390.athena.vision.spec.VisionFrame;

/**
 * Mutable simulated camera frame source.
 */
public final class SimVisionCamera {
    private final CameraSpec spec;
    private VisionFrame frame = VisionFrame.noTarget();

    /**
     * Creates a simulated camera.
     *
     * @param spec camera spec
     */
    public SimVisionCamera(CameraSpec spec) {
        this.spec = spec;
    }

    /**
     * Returns camera spec.
     *
     * @return camera spec
     */
    public CameraSpec spec() {
        return spec;
    }

    /**
     * Returns current frame.
     *
     * @return current frame
     */
    public VisionFrame frame() {
        return frame;
    }

    /**
     * Sets current frame.
     *
     * @param frame vision frame
     * @return this camera
     */
    public SimVisionCamera frame(VisionFrame frame) {
        this.frame = frame == null ? VisionFrame.noTarget() : frame;
        return this;
    }
}
