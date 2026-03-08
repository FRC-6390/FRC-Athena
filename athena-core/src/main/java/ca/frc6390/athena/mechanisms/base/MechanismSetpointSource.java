package ca.frc6390.athena.mechanisms;

import java.util.Objects;

/**
 * Selects where a controller gets its setpoint signal from.
 */
public final class MechanismSetpointSource {
    public enum Kind {
        SETPOINT,
        INPUT
    }

    public static final MechanismSetpointSource Setpoint = new MechanismSetpointSource(Kind.SETPOINT, null);

    private final Kind kind;
    private final String inputKey;

    private MechanismSetpointSource(Kind kind, String inputKey) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.inputKey = inputKey;
    }

    public static MechanismSetpointSource input(String key) {
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("setpoint input key cannot be blank");
        }
        return new MechanismSetpointSource(Kind.INPUT, key.trim());
    }

    public Kind kind() {
        return kind;
    }

    public String inputKey() {
        return inputKey;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof MechanismSetpointSource other)) {
            return false;
        }
        return kind == other.kind && Objects.equals(inputKey, other.inputKey);
    }

    @Override
    public int hashCode() {
        return Objects.hash(kind, inputKey);
    }

    @Override
    public String toString() {
        return switch (kind) {
            case SETPOINT -> "setpoint";
            case INPUT -> "input:" + inputKey;
        };
    }
}
