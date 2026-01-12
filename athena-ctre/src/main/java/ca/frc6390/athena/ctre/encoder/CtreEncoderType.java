package ca.frc6390.athena.ctre.encoder;

import ca.frc6390.athena.hardware.encoder.EncoderType;

/**
 * CTRE encoder types.
 */
public enum CtreEncoderType implements EncoderType {
    CANCODER("ctre:cancoder"),
    TALON_FX("ctre:talonfx-integrated");

    private final String key;

    CtreEncoderType(String key) {
        this.key = key;
    }

    @Override
    public String getKey() {
        return key;
    }
}
