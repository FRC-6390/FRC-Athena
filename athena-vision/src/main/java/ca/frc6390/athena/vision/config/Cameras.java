package ca.frc6390.athena.vision.config;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.vision.ref.CameraDevice;
import ca.frc6390.athena.vision.ref.GenericCameraDevice;
import ca.frc6390.athena.vision.ref.HeliosDevice;
import ca.frc6390.athena.vision.ref.LimelightDevice;
import ca.frc6390.athena.vision.ref.PhotonVisionDevice;

/**
 * Entry points for camera declarations.
 */
public final class Cameras {
    private Cameras() {
    }

    /**
     * Creates a camera declaration.
     *
     * @param kind camera kind
     * @param name camera name
     * @return camera device
     */
    public static CameraDevice camera(CameraKind kind, String name) {
        return new GenericCameraDevice(kind, name);
    }

    /**
     * Creates a Limelight camera declaration.
     *
     * @param name camera name
     * @return Limelight device
     */
    public static LimelightDevice limelight(String name) {
        return new LimelightDevice(name);
    }

    /**
     * Creates a PhotonVision camera declaration.
     *
     * @param name camera name
     * @return PhotonVision device
     */
    public static PhotonVisionDevice photonVision(String name) {
        return new PhotonVisionDevice(name);
    }

    /**
     * Creates a HeliOS camera declaration.
     *
     * @param address camera name or address
     * @return HeliOS device
     */
    public static HeliosDevice helios(String address) {
        return new HeliosDevice(address);
    }

}
