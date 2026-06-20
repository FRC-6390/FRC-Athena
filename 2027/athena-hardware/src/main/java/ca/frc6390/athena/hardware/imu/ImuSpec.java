package ca.frc6390.athena.hardware.imu;

import java.util.Objects;

import ca.frc6390.athena.api.hardware.ImuKind;

/**
 * Immutable normalized IMU declaration.
 *
 * @param ownerPath owning subsystem path
 * @param name IMU name
 * @param kind IMU kind
 * @param id device id
 * @param canbus CAN bus name
 * @param mountPose robot-relative mount pose
 */
public record ImuSpec(
        String ownerPath,
        String name,
        ImuKind kind,
        int id,
        String canbus,
        ImuMountPose mountPose) {
    public ImuSpec {
        ownerPath = ownerPath == null || ownerPath.isBlank() ? "robot" : ownerPath;
        name = name == null || name.isBlank() ? "imu" : name;
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        mountPose = mountPose == null ? ImuMountPose.identity() : mountPose;
    }

    /**
     * Returns a dotted path useful in validation errors.
     *
     * @return IMU path
     */
    public String path() {
        return ownerPath + "." + name;
    }
}
