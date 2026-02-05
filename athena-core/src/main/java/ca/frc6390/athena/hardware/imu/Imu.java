package ca.frc6390.athena.hardware.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import ca.frc6390.athena.core.RobotSendableSystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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

    // Optional linear acceleration accessors (default to zero)
    default double getAccelX() { return 0.0; }
    default double getAccelY() { return 0.0; }
    default double getAccelZ() { return 0.0; }

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

    // Cached accessors for shuffleboard/logging (override in adapters to return cached values)
    default Rotation2d getCachedRoll() { return getRoll(); }
    default Rotation2d getCachedPitch() { return getPitch(); }
    default Rotation2d getCachedYaw() { return getYaw(); }
    default Rotation2d getCachedVelocityX() { return getVelocityX(); }
    default Rotation2d getCachedVelocityY() { return getVelocityY(); }
    default Rotation2d getCachedVelocityZ() { return getVelocityZ(); }
    default double getCachedAccelX() { return getAccelX(); }
    default double getCachedAccelY() { return getAccelY(); }
    default double getCachedAccelZ() { return getAccelZ(); }
    default boolean isCachedConnected() { return isConnected(); }

    // Simulation hooks (default no-op)
    default void setSimulatedHeading(Rotation2d yaw, Rotation2d angularVelocityZ) {}
    default void setSimulatedReadings(Rotation2d yaw, Rotation2d pitch, Rotation2d roll,
                                      Rotation2d velX, Rotation2d velY, Rotation2d velZ) {}
    default void disableSimulatedReadings() {}

    ImuConfig getConfig();

    @Override
    default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, RobotSendableSystem.SendableLevel level) {
        layout.addDouble("Yaw (deg)", () -> {
            Rotation2d yaw = getCachedYaw();
            return yaw != null ? yaw.getDegrees() : 0.0;
        });
        layout.addDouble("Pitch (deg)", () -> {
            Rotation2d pitch = getCachedPitch();
            return pitch != null ? pitch.getDegrees() : 0.0;
        });
        layout.addDouble("Roll (deg)", () -> {
            Rotation2d roll = getCachedRoll();
            return roll != null ? roll.getDegrees() : 0.0;
        });
        layout.addDouble("Velocity X (deg/s)", () -> {
            Rotation2d vel = getCachedVelocityX();
            return vel != null ? vel.getDegrees() : 0.0;
        });
        layout.addDouble("Velocity Y (deg/s)", () -> {
            Rotation2d vel = getCachedVelocityY();
            return vel != null ? vel.getDegrees() : 0.0;
        });
        layout.addDouble("Velocity Z (deg/s)", () -> {
            Rotation2d vel = getCachedVelocityZ();
            return vel != null ? vel.getDegrees() : 0.0;
        });
        layout.addDouble("Accel X", this::getCachedAccelX);
        layout.addDouble("Accel Y", this::getCachedAccelY);
        layout.addDouble("Accel Z", this::getCachedAccelZ);
        layout.addBoolean("Connected", this::isCachedConnected);
        if (level.equals(RobotSendableSystem.SendableLevel.DEBUG)) {
            layout.add("Inverted", builder ->
                    builder.addBooleanProperty("Inverted", this::isInverted, this::setInverted));
            layout.addDouble("Max Linear Speed", this::getMaxLinearSpeed);
            layout.addDouble("Max Radial Speed", this::getMaxRadialSpeed);
            layout.add("Max Speed Window (s)",
                    builder -> builder.addDoubleProperty(
                            "Max Speed Window (s)",
                            this::getMaxSpeedWindowSeconds,
                            this::setMaxSpeedWindowSeconds));
            layout.add("Reset Max Speeds", new InstantCommand(this::resetMaxSpeedWindow))
                    .withWidget(BuiltInWidgets.kCommand);
            layout.addDouble("CAN ID", () -> {
                ImuConfig cfg = getConfig();
                return cfg != null ? cfg.id : 0.0;
            });
            layout.addString("CAN Bus", () -> {
                ImuConfig cfg = getConfig();
                return cfg != null && cfg.canbus != null ? cfg.canbus : "";
            });
            layout.addString("Type", () -> {
                ImuConfig cfg = getConfig();
                return cfg != null && cfg.type != null ? cfg.type.getKey() : "unknown";
            });
        }
        return layout;
    }
}
