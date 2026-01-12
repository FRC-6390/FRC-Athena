package ca.frc6390.athena.hardware.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import ca.frc6390.athena.core.RobotSendableSystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

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
    default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, RobotSendableSystem.SendableLevel level) {
        return layout;
    }
}
