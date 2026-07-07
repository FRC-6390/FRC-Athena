package ca.frc6390.athena.vision.config;

import java.util.function.Consumer;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.vision.ref.CameraRef;
import ca.frc6390.athena.vision.ref.GenericCameraRef;
import ca.frc6390.athena.vision.ref.HeliOS;
import ca.frc6390.athena.vision.ref.HeliOSRef;
import ca.frc6390.athena.vision.ref.Limelight;
import ca.frc6390.athena.vision.ref.LimelightRef;
import ca.frc6390.athena.vision.ref.Photon;
import ca.frc6390.athena.vision.ref.PhotonRef;
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

    /**
     * Creates a generic camera ref.
     *
     * @param kind camera kind
     * @param name camera name
     * @return camera ref
     */
    public static CameraRef camera(CameraKind kind, String name) {
        return new GenericCameraRef(kind, name);
    }

    /**
     * Creates a Limelight camera ref.
     *
     * @param name camera name
     * @return Limelight ref
     */
    public static LimelightRef limelight(String name) {
        return Limelight.camera(name);
    }

    /**
     * Creates a PhotonVision camera ref.
     *
     * @param name camera name
     * @return Photon ref
     */
    public static PhotonRef photon(String name) {
        return Photon.camera(name);
    }

    /**
     * Creates a HeliOS camera ref.
     *
     * @param address camera name or address
     * @return HeliOS ref
     */
    public static HeliOSRef helios(String address) {
        return HeliOS.camera(address);
    }

    /**
     * Creates a simulation camera ref.
     *
     * @param name camera name
     * @return camera ref
     */
    public static CameraRef sim(String name) {
        return camera(AthenaCamera.SIM, name);
    }
}
