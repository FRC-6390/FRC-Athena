package ca.frc6390.athena.vision.config;

import java.util.function.Consumer;

import ca.frc6390.athena.vision.spec.CameraSpec;

/**
 * Entry points for camera declarations.
 */
public final class Cameras {
    private Cameras() {
    }

    /**
     * Creates and lowers a single named camera declaration.
     *
     * @param name camera name in the robot model
     * @param configure camera configuration
     * @return camera spec
     */
    public static CameraSpec camera(String name, Consumer<CameraConfig> configure) {
        CameraConfig config = CameraConfig.create();
        if (configure != null) {
            configure.accept(config);
        }
        return config.toSpec("vision", name);
    }
}
