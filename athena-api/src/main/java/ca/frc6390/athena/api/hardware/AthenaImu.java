package ca.frc6390.athena.api.hardware;

/**
 * Common built-in IMU kinds known to Athena.
 */
public enum AthenaImu implements ImuKind {
    /** CTRE Pigeon 2 IMU. */
    PIGEON_2("ctre:pigeon-2"),

    /** Studica NavX IMU. */
    NAVX("studica:navx"),

    /** Simulation-only IMU backend. */
    SIM("sim:imu");

    private final String key;

    AthenaImu(String key) {
        this.key = key;
    }

    @Override
    public String key() {
        return key;
    }
}
