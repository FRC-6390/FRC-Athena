package ca.frc6390.athena.rev.motor;

import ca.frc6390.athena.hardware.motor.MotorControllerType;

/**
 * REV motor controller types.
 */
public enum RevMotorControllerType implements MotorControllerType {
    SPARK_MAX_BRUSHED("rev:sparkmax:brushed"),
    SPARK_MAX_BRUSHLESS("rev:sparkmax:brushless"),
    SPARK_FLEX_BRUSHED("rev:sparkflex:brushed"),
    SPARK_FLEX_BRUSHLESS("rev:sparkflex:brushless");

    private final String key;

    RevMotorControllerType(String key) {
        this.key = key;
    }

    @Override
    public String getKey() {
        return key;
    }
}
