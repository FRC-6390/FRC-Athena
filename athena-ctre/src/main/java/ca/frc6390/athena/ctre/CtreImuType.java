package ca.frc6390.athena.ctre;

import ca.frc6390.athena.hardware.ImuType;

/**
 * CTRE IMU types.
 */
public enum CtreImuType implements ImuType {
    PIGEON2("ctre:pigeon2");

    private final String key;

    CtreImuType(String key) {
        this.key = key;
    }

    @Override
    public String getKey() {
        return key;
    }
}
