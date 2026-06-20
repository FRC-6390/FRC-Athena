package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.hardware.backend.ImuDevice;
import ca.frc6390.athena.hardware.imu.ImuSpec;

/**
 * In-memory simulation IMU.
 *
 * @param spec normalized IMU spec
 * @param yawDegrees simulated yaw
 */
public record SimImuDevice(ImuSpec spec, double yawDegrees) implements ImuDevice {
}
