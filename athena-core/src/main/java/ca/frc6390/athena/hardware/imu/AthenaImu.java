package ca.frc6390.athena.hardware.imu;

/**
 * Team-facing IMU catalog keyed by vendor string IDs.
 */
public enum AthenaImu {
    PIGEON2("ctre:pigeon2"),
    NAVX("studica:navx");

    private final String key;

    AthenaImu(String key) {
        this.key = key;
    }

    public ImuType resolve() {
        return ImuRegistry.get().imu(key);
    }
}
