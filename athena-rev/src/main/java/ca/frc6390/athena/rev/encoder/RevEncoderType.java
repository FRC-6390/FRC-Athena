package ca.frc6390.athena.rev.encoder;

import ca.frc6390.athena.hardware.encoder.EncoderType;

/**
 * REV encoder types.
 */
public enum RevEncoderType implements EncoderType {
    SPARK_MAX("rev:sparkmax"),
    SPARK_FLEX("rev:sparkflex");

    private final String key;

    RevEncoderType(String key) {
        this.key = key;
    }

    @Override
    public String getKey() {
        return key;
    }
}
