package ca.frc6390.athena.vision.config;

import java.util.Objects;

import ca.frc6390.athena.api.hardware.CameraId;
import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.vision.spec.CameraMountPose;
import ca.frc6390.athena.vision.spec.CameraSpec;

/**
 * Student-facing camera declaration.
 */
public final class CameraConfig {
    private CameraKind kind;
    private String cameraName = "camera";
    private CameraMountPose mountPose = CameraMountPose.identity();

    private CameraConfig() {
    }

    /**
     * Creates an empty camera config.
     *
     * @return camera config
     */
    public static CameraConfig create() {
        return new CameraConfig();
    }

    /**
     * Sets camera identity.
     *
     * @param kind camera kind
     * @param name camera or NetworkTables name
     * @return this config
     */
    public CameraConfig hardware(CameraKind kind, String name) {
        this.kind = Objects.requireNonNull(kind, "kind");
        cameraName = name == null || name.isBlank() ? "camera" : name;
        return this;
    }

    /**
     * Sets camera identity from a reusable alias.
     *
     * @param cameraId camera identity
     * @return this config
     */
    public CameraConfig hardware(CameraId cameraId) {
        Objects.requireNonNull(cameraId, "cameraId");
        return hardware(cameraId.kind(), cameraId.name());
    }

    /**
     * Sets robot-relative camera mount pose in meters and degrees.
     *
     * @param xMeters forward-positive position
     * @param yMeters left-positive position
     * @param zMeters up-positive position
     * @param yawDegrees yaw offset
     * @param pitchDegrees pitch offset
     * @param rollDegrees roll offset
     * @return this config
     */
    public CameraConfig mountPose(
            double xMeters,
            double yMeters,
            double zMeters,
            double yawDegrees,
            double pitchDegrees,
            double rollDegrees) {
        mountPose = new CameraMountPose(xMeters, yMeters, zMeters, yawDegrees, pitchDegrees, rollDegrees);
        return this;
    }

    /**
     * Lowers this declaration into an immutable camera spec.
     *
     * @param ownerPath owner path
     * @param name local camera name
     * @return camera spec
     */
    public CameraSpec toSpec(String ownerPath, String name) {
        if (kind == null) {
            throw new IllegalStateException("Camera hardware kind is required for " + ownerPath + "." + name);
        }
        return new CameraSpec(ownerPath, name, kind, cameraName, mountPose);
    }
}
