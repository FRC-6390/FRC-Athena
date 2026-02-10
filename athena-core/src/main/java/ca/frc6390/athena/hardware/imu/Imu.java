package ca.frc6390.athena.hardware.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotNetworkTables;

/**
 * Vendor-agnostic IMU interface for the new vendordep system.
 */
public interface Imu extends RobotSendableSystem.RobotSendableDevice {
    Rotation2d getRoll();

    Rotation2d getPitch();

    Rotation2d getYaw();

    void setInverted(boolean inverted);

    default boolean isInverted() { return false; }

    default boolean isConnected() { return true; }

    // Optional velocity accessors (default to zero)
    default Rotation2d getVelocityX() { return new Rotation2d(); }
    default Rotation2d getVelocityY() { return new Rotation2d(); }
    default Rotation2d getVelocityZ() { return new Rotation2d(); }

    // Optional angular acceleration accessors (default to zero)
    default double getAngularAccelerationZRadiansPerSecondSquared() { return 0.0; }
    default double getAngularAccelerationZDegreesPerSecondSquared() {
        return Math.toDegrees(getAngularAccelerationZRadiansPerSecondSquared());
    }

    // Optional linear acceleration accessors (default to zero)
    default double getAccelerationX() { return 0.0; }
    default double getAccelerationY() { return 0.0; }
    default double getAccelerationZ() { return 0.0; }

    // Optional max speed tracking helpers (default to no-op)
    default double getMaxLinearSpeed() { return 0.0; }
    default double getMaxRadialSpeed() { return 0.0; }
    default double getMaxSpeedWindowSeconds() { return 5.0; }
    default void setMaxSpeedWindowSeconds(double windowSeconds) {}
    default void resetMaxSpeedWindow() {}

    // Optional heading reset hook (default no-op)
    default void setYaw(Rotation2d yaw) {}
    default void setYaw(double yawDegrees) { setYaw(Rotation2d.fromDegrees(yawDegrees)); }

    // Virtual axis helpers (default no-op implementations for compatibility)
    default void addVirtualAxis(String name, java.util.function.Supplier<Rotation2d> supplier) {}
    default Rotation2d getVirtualAxis(String name) { return new Rotation2d(); }
    default void setVirtualAxis(String name, Rotation2d value) {}

    // Optional periodic update hook (default no-op)
    default void update() {}

    // Cached accessors for logging/dashboard (override in adapters to return cached values)
    default Rotation2d getCachedRoll() { return getRoll(); }
    default Rotation2d getCachedPitch() { return getPitch(); }
    default Rotation2d getCachedYaw() { return getYaw(); }
    default Rotation2d getCachedVelocityX() { return getVelocityX(); }
    default Rotation2d getCachedVelocityY() { return getVelocityY(); }
    default Rotation2d getCachedVelocityZ() { return getVelocityZ(); }
    default double getCachedAngularAccelerationZRadiansPerSecondSquared() {
        return getAngularAccelerationZRadiansPerSecondSquared();
    }
    default double getCachedAngularAccelerationZDegreesPerSecondSquared() {
        return Math.toDegrees(getCachedAngularAccelerationZRadiansPerSecondSquared());
    }
    default double getCachedAccelerationX() { return getAccelerationX(); }
    default double getCachedAccelerationY() { return getAccelerationY(); }
    default double getCachedAccelerationZ() { return getAccelerationZ(); }
    default boolean isCachedConnected() { return isConnected(); }

    // Simulation hooks (default no-op)
    default void setSimulatedHeading(Rotation2d yaw, Rotation2d angularVelocityZ) {}
    default void setSimulatedReadings(Rotation2d yaw, Rotation2d pitch, Rotation2d roll,
                                      Rotation2d velX, Rotation2d velY, Rotation2d velZ) {}
    default void disableSimulatedReadings() {}

    ImuConfig getConfig();

    @Override
    default RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        Rotation2d yaw = getCachedYaw();
        Rotation2d pitch = getCachedPitch();
        Rotation2d roll = getCachedRoll();
        Rotation2d velX = getCachedVelocityX();
        Rotation2d velY = getCachedVelocityY();
        Rotation2d velZ = getCachedVelocityZ();

        node.putDouble("yawDeg", yaw != null ? yaw.getDegrees() : 0.0);
        node.putDouble("pitchDeg", pitch != null ? pitch.getDegrees() : 0.0);
        node.putDouble("rollDeg", roll != null ? roll.getDegrees() : 0.0);
        node.putDouble("velXDegPerSec", velX != null ? velX.getDegrees() : 0.0);
        node.putDouble("velYDegPerSec", velY != null ? velY.getDegrees() : 0.0);
        node.putDouble("velZDegPerSec", velZ != null ? velZ.getDegrees() : 0.0);
        node.putDouble("accelX", getCachedAccelerationX());
        node.putDouble("accelY", getCachedAccelerationY());
        node.putDouble("accelZ", getCachedAccelerationZ());
        node.putBoolean("connected", isCachedConnected());

        if (nt.enabled(RobotNetworkTables.Flag.HW_IMU_TUNING_WIDGETS)) {
            node.putBoolean("inverted", isInverted());
            node.putDouble("maxLinearSpeed", getMaxLinearSpeed());
            node.putDouble("maxRadialSpeed", getMaxRadialSpeed());
            node.putDouble("maxSpeedWindowSec", getMaxSpeedWindowSeconds());
            node.putDouble("angularAccelZDegPerSec2", getCachedAngularAccelerationZDegreesPerSecondSquared());

            ImuConfig cfg = getConfig();
            if (cfg != null) {
                node.putDouble("canId", cfg.id);
                node.putString("canbus", cfg.canbus != null ? cfg.canbus : "");
                node.putString("type", cfg.type != null ? cfg.type.getKey() : "unknown");
            }
        }

        return node;
    }
}
