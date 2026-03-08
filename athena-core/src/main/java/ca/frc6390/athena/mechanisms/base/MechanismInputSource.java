package ca.frc6390.athena.mechanisms;

import java.util.Objects;

/**
 * Selects which measurement channel a control profile should use.
 */
public final class MechanismInputSource {
    public enum Kind {
        POSITION,
        VELOCITY,
        ABSOLUTE,
        INPUT
    }

    public static final MechanismInputSource Position = new MechanismInputSource(Kind.POSITION, null, null);
    public static final MechanismInputSource Velocity = new MechanismInputSource(Kind.VELOCITY, null, null);
    public static final MechanismInputSource Absolute = new MechanismInputSource(Kind.ABSOLUTE, null, null);

    private final Kind kind;
    private final String encoderId;
    private final String inputKey;

    private MechanismInputSource(Kind kind, String encoderId, String inputKey) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.encoderId = encoderId;
        this.inputKey = inputKey;
    }

    public static MechanismInputSource position(String encoderId) {
        return named(Kind.POSITION, encoderId);
    }

    public static MechanismInputSource velocity(String encoderId) {
        return named(Kind.VELOCITY, encoderId);
    }

    public static MechanismInputSource absolute(String encoderId) {
        return named(Kind.ABSOLUTE, encoderId);
    }

    public static MechanismInputSource input(String key) {
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("input source key cannot be blank");
        }
        return new MechanismInputSource(Kind.INPUT, null, key.trim());
    }

    public Kind kind() {
        return kind;
    }

    public String encoderId() {
        return encoderId;
    }

    public String inputKey() {
        return inputKey;
    }

    private static MechanismInputSource named(Kind kind, String encoderId) {
        if (encoderId == null || encoderId.isBlank()) {
            throw new IllegalArgumentException("encoder source id cannot be blank");
        }
        return new MechanismInputSource(kind, encoderId.trim(), null);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof MechanismInputSource other)) {
            return false;
        }
        return kind == other.kind
                && Objects.equals(encoderId, other.encoderId)
                && Objects.equals(inputKey, other.inputKey);
    }

    @Override
    public int hashCode() {
        return Objects.hash(kind, encoderId, inputKey);
    }

    @Override
    public String toString() {
        return switch (kind) {
            case POSITION -> encoderId != null ? "position:" + encoderId : "position";
            case VELOCITY -> encoderId != null ? "velocity:" + encoderId : "velocity";
            case ABSOLUTE -> encoderId != null ? "absolute:" + encoderId : "absolute";
            case INPUT -> "input:" + inputKey;
        };
    }
}
