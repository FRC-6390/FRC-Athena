package ca.frc6390.athena.hardware.imu;

import java.util.Objects;

import ca.frc6390.athena.api.hardware.ImuId;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.hardware.ref.ImuRef;

/**
 * Student-facing IMU declaration.
 */
public final class ImuConfig {
    private ImuKind kind;
    private int id;
    private String canbus = "rio";
    private ImuMountPose mountPose = ImuMountPose.identity();

    private ImuConfig() {
    }

    /**
     * Creates an empty IMU config.
     *
     * @return IMU config
     */
    public static ImuConfig create() {
        return new ImuConfig();
    }

    /**
     * Sets hardware identity.
     *
     * @param kind IMU kind
     * @param id device id
     * @return this config
     */
    public ImuConfig hardware(ImuKind kind, int id) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.id = id;
        return this;
    }

    /**
     * Sets hardware identity from a reusable alias.
     *
     * @param imuId IMU identity
     * @return this config
     */
    public ImuConfig hardware(ImuId imuId) {
        Objects.requireNonNull(imuId, "imuId");
        return hardware(imuId.kind(), imuId.id()).canbus(imuId.canbus());
    }

    /**
     * Sets hardware identity from an IMU ref.
     *
     * @param imu IMU ref
     * @return this config
     */
    public ImuConfig hardware(ImuRef imu) {
        Objects.requireNonNull(imu, "imu");
        return hardware(imu.kind(), imu.id())
                .canbus(imu.canbus())
                .mountPose(imu.mountPose().yawDegrees(), imu.mountPose().pitchDegrees(), imu.mountPose().rollDegrees());
    }

    /**
     * Sets CAN bus.
     *
     * @param canbus bus name
     * @return this config
     */
    public ImuConfig canbus(String canbus) {
        this.canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        return this;
    }

    /**
     * Sets robot-relative mount pose.
     *
     * @param yawDegrees yaw offset
     * @param pitchDegrees pitch offset
     * @param rollDegrees roll offset
     * @return this config
     */
    public ImuConfig mountPose(double yawDegrees, double pitchDegrees, double rollDegrees) {
        mountPose = new ImuMountPose(yawDegrees, pitchDegrees, rollDegrees);
        return this;
    }

    /**
     * Lowers this declaration into an immutable spec.
     *
     * @param ownerPath owner path
     * @param name IMU name
     * @return IMU spec
     */
    public ImuSpec toSpec(String ownerPath, String name) {
        if (kind == null) {
            throw new IllegalStateException("IMU hardware kind is required for " + ownerPath + "." + name);
        }
        return new ImuSpec(ownerPath, name, kind, id, canbus, mountPose);
    }
}
