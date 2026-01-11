package ca.frc6390.athena.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Vendor-agnostic IMU interface for the new vendordep system.
 */
public interface Imu {
    Rotation2d getRoll();

    Rotation2d getPitch();

    Rotation2d getYaw();

    void setInverted(boolean inverted);

    ImuConfig getConfig();
}
