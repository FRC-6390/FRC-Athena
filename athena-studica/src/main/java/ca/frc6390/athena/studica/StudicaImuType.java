package ca.frc6390.athena.studica;

import ca.frc6390.athena.hardware.ImuType;

/**
 * Studica IMU types (NavX family).
 */
public enum StudicaImuType implements ImuType {
    NAVX("studica:navx");

    private final String key;

    StudicaImuType(String key) {
        this.key = key;
    }

    @Override
    public String getKey() {
        return key;
    }
}
