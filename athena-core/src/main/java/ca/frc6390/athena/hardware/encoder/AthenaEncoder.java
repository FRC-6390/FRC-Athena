package ca.frc6390.athena.hardware.encoder;

/**
 * Team-facing encoder catalog keyed by vendor string IDs. Passing these enums keeps user code
 * vendor-agnostic while still resolving to the correct vendordep types.
 */
public enum AthenaEncoder {
    CANCODER("ctre:cancoder"),
    SPARK_MAX("rev:sparkmax"),
    SPARK_FLEX("rev:sparkflex");

    private final String key;

    AthenaEncoder(String key) {
        this.key = key;
    }

    public EncoderType resolve() {
        return EncoderRegistry.get().encoder(key);
    }
}
