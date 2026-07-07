package ca.frc6390.athena.hardware.ref;

import java.util.Locale;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.ImuId;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.hardware.imu.ImuMountPose;

/**
 * Reusable IMU declaration.
 */
public record ImuRef(
        ImuKind kind,
        int id,
        String canbus,
        ImuMountPose mountPose) {
    public static ImuRef of(ImuKind kind, int id) {
        return new ImuRef(kind, id, "rio", ImuMountPose.identity());
    }

    public static ImuRef of(ImuId id) {
        Objects.requireNonNull(id, "id");
        return of(id.kind(), id.id()).canbus(id.canbus());
    }

    public ImuRef {
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        mountPose = mountPose == null ? ImuMountPose.identity() : mountPose;
    }

    public ImuId idRef() {
        return new ImuId(kind, id, canbus);
    }

    public ImuRef canbus(String canbus) {
        return new ImuRef(kind, id, canbus, mountPose);
    }

    public ImuRef mountPose(double yawDegrees, double pitchDegrees, double rollDegrees) {
        return mountPose(new ImuMountPose(yawDegrees, pitchDegrees, rollDegrees));
    }

    public ImuRef mountPose(ImuMountPose mountPose) {
        return new ImuRef(kind, id, canbus, mountPose);
    }

    public String defaultName() {
        return sanitize(kind.key()) + "_" + id;
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }
}
