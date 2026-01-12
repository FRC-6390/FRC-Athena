package ca.frc6390.athena.studica.imu;

import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import ca.frc6390.athena.hardware.factory.ImuFactory;

/**
 * Studica/NavX IMU factory wiring.
 */
public class StudicaImuFactory implements ImuFactory {
    @Override
    public boolean supports(ImuType type) {
        return type instanceof StudicaImuType;
    }

    @Override
    public Imu create(ImuConfig config) {
        return StudicaImu.fromConfig(config);
    }
}
