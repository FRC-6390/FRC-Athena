package ca.frc6390.athena.ctre.motor;

import ca.frc6390.athena.hardware.motor.MotorControllerType;

/**
 * CTRE motor controller types.
 */
public enum CtreMotorControllerType implements MotorControllerType {
    TALON_FX("ctre:talonfx");

    private final String key;

    CtreMotorControllerType(String key) {
        this.key = key;
    }

    @Override
    public String getKey() {
        return key;
    }
}
