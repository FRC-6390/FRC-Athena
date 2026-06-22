package ca.frc6390.athena.api.hardware;

import java.util.Objects;

/**
 * Reusable camera identity for robot-wide hardware maps.
 *
 * @param kind camera hardware kind
 * @param name camera or NetworkTables name
 */
public record CameraId(CameraKind kind, String name) {
    /**
     * Creates a camera identity.
     *
     * @param kind camera kind
     * @param name camera or NetworkTables name
     * @return camera identity
     */
    public static CameraId of(CameraKind kind, String name) {
        return new CameraId(kind, name);
    }

    public CameraId {
        Objects.requireNonNull(kind, "kind");
        name = name == null || name.isBlank() ? "camera" : name;
    }
}
