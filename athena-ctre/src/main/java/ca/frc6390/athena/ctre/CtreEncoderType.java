package ca.frc6390.athena.ctre;

import ca.frc6390.athena.hardware.EncoderType;

/**
 * CTRE encoder types.
 */
public enum CtreEncoderType implements EncoderType {
    CANCODER("ctre:cancoder");

    private final String key;

    CtreEncoderType(String key) {
        this.key = key;
    }

    @Override
    public String getKey() {
        return key;
    }
}
