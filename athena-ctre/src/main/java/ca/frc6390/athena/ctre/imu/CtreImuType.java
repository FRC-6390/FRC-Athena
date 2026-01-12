package ca.frc6390.athena.ctre.imu;

import ca.frc6390.athena.hardware.imu.ImuType;

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
