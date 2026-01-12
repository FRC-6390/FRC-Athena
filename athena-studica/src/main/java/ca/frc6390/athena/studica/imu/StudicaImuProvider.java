package ca.frc6390.athena.studica.imu;

import ca.frc6390.athena.hardware.imu.ImuRegistry;

/**
 * Registers Studica/NavX IMU types with the IMU registry.
 */
public class StudicaImuProvider implements ImuRegistry.Provider {
    @Override
    public void register(ImuRegistry registry) {
        registry.add(StudicaImuType.NAVX);
    }
}
