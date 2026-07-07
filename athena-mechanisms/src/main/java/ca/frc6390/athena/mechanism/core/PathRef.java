package ca.frc6390.athena.mechanism.core;

import java.util.Objects;
import java.util.OptionalDouble;

/**
 * Autonomous path request that can be composed anywhere a mechanism state can be used.
 */
public record PathRef(String source, String name, double expectedSeconds) implements MechanismState {
    public PathRef {
        source = source == null || source.isBlank() ? "path" : source.trim();
        name = name == null || name.isBlank() ? "default" : name.trim();
    }

    public PathRef(String source, String name) {
        this(source, name, Double.NaN);
    }

    public PathRef seconds(double seconds) {
        return new PathRef(source, name, seconds);
    }

    public OptionalDouble expectedDurationSeconds() {
        return Double.isFinite(expectedSeconds) ? OptionalDouble.of(expectedSeconds) : OptionalDouble.empty();
    }

    public String key() {
        return source + ":" + name;
    }

    public boolean samePath(PathRef other) {
        return other != null
                && Objects.equals(source, other.source)
                && Objects.equals(name, other.name);
    }
}
