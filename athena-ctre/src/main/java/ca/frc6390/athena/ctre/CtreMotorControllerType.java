package ca.frc6390.athena.ctre;

import ca.frc6390.athena.hardware.MotorControllerType;

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
