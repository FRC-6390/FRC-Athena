package ca.frc6390.athena.hardware.factory;

import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;

/**
 * Service provider interface for creating IMUs from an {@link ImuConfig}.
 */
public interface ImuFactory {
    boolean supports(ImuType type);

    Imu create(ImuConfig config);
}
