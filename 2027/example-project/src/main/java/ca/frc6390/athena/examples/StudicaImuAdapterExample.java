package ca.frc6390.athena.examples;

import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.hardware.backend.ImuDevice;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuSpec;
import ca.frc6390.athena.vendor.ctre.CtreImuBackend;
import ca.frc6390.athena.vendor.studica.StudicaImuBackend;

/**
 * IMU vendor adapter examples.
 */
public final class StudicaImuAdapterExample {
    /**
     * NavX IMU declaration using the common Athena hardware key.
     */
    public static final ImuSpec NAVX = ImuConfig.create()
            .hardware(AthenaImu.NAVX, 0)
            .mountPose(0.0, 0.0, 0.0)
            .toSpec("robot", "navx");

    /**
     * Pigeon2 IMU declaration using the common Athena hardware key.
     */
    public static final ImuSpec PIGEON = ImuConfig.create()
            .hardware(AthenaImu.PIGEON_2, 30)
            .canbus("canivore")
            .mountPose(0.0, 0.0, 0.0)
            .toSpec("robot", "pigeon");

    /**
     * Creates a real Studica/NavX device without importing Studica classes into
     * generic hardware declarations.
     *
     * @return IMU device
     */
    public static ImuDevice createDevice() {
        return new StudicaImuBackend().create(NAVX);
    }

    /**
     * Creates a real CTRE Pigeon2 device without importing Phoenix classes into
     * generic hardware declarations.
     *
     * @return IMU device
     */
    public static ImuDevice createPigeonDevice() {
        return new CtreImuBackend().create(PIGEON);
    }

    private StudicaImuAdapterExample() {
    }
}
