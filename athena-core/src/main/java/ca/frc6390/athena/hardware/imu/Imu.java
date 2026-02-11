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

    default Rotation2d getRoll(boolean poll) {
        if (poll) {
            update();
        }
        return getRoll();
    }

    default Rotation2d getPitch(boolean poll) {
        if (poll) {
            update();
        }
        return getPitch();
    }

    default Rotation2d getYaw(boolean poll) {
        if (poll) {
            update();
        }
        return getYaw();
    }

    void setInverted(boolean inverted);

    default boolean isInverted() { return false; }

    default boolean isConnected() { return true; }
    default boolean isConnected(boolean poll) {
        if (poll) {
            update();
        }
        return isConnected();
    }

    // Optional velocity accessors (default to zero)
    default Rotation2d getVelocityX() { return new Rotation2d(); }
    default Rotation2d getVelocityY() { return new Rotation2d(); }
    default Rotation2d getVelocityZ() { return new Rotation2d(); }
    default Rotation2d getVelocityX(boolean poll) {
        if (poll) {
            update();
        }
        return getVelocityX();
    }
    default Rotation2d getVelocityY(boolean poll) {
        if (poll) {
            update();
        }
        return getVelocityY();
    }
    default Rotation2d getVelocityZ(boolean poll) {
        if (poll) {
            update();
        }
        return getVelocityZ();
    }

    // Optional angular acceleration accessors (default to zero)
    default double getAngularAccelerationZRadiansPerSecondSquared() { return 0.0; }
    default double getAngularAccelerationZDegreesPerSecondSquared() {
        return Math.toDegrees(getAngularAccelerationZRadiansPerSecondSquared());
    }

    // Optional linear acceleration accessors (default to zero)
    default double getAccelerationX() { return 0.0; }
    default double getAccelerationY() { return 0.0; }
    default double getAccelerationZ() { return 0.0; }
    default double getAccelerationX(boolean poll) {
        if (poll) {
            update();
        }
        return getAccelerationX();
    }
    default double getAccelerationY(boolean poll) {
        if (poll) {
            update();
        }
        return getAccelerationY();
    }
    default double getAccelerationZ(boolean poll) {
        if (poll) {
            update();
        }
        return getAccelerationZ();
    }

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

        Rotation2d yaw = getYaw();
        Rotation2d pitch = getPitch();
        Rotation2d roll = getRoll();
        Rotation2d velX = getVelocityX();
        Rotation2d velY = getVelocityY();
        Rotation2d velZ = getVelocityZ();

        node.putDouble("yawDeg", yaw != null ? yaw.getDegrees() : 0.0);
        node.putDouble("pitchDeg", pitch != null ? pitch.getDegrees() : 0.0);
        node.putDouble("rollDeg", roll != null ? roll.getDegrees() : 0.0);
        node.putDouble("velXDegPerSec", velX != null ? velX.getDegrees() : 0.0);
        node.putDouble("velYDegPerSec", velY != null ? velY.getDegrees() : 0.0);
        node.putDouble("velZDegPerSec", velZ != null ? velZ.getDegrees() : 0.0);
        node.putDouble("accelX", getAccelerationX());
        node.putDouble("accelY", getAccelerationY());
        node.putDouble("accelZ", getAccelerationZ());
        node.putBoolean("connected", isConnected());

        if (nt.enabled(RobotNetworkTables.Flag.HW_IMU_TUNING_WIDGETS)) {
            node.putBoolean("inverted", isInverted());
            node.putDouble("maxLinearSpeed", getMaxLinearSpeed());
            node.putDouble("maxRadialSpeed", getMaxRadialSpeed());
            node.putDouble("maxSpeedWindowSec", getMaxSpeedWindowSeconds());
            node.putDouble("angularAccelZDegPerSec2", getAngularAccelerationZDegreesPerSecondSquared());

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
