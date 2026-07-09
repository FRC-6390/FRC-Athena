package ca.frc6390.athena.mechanism.core;

import java.util.Objects;
import java.util.OptionalDouble;

/**
 * Autonomous path request that can be composed anywhere a mechanism Action can be used.
 */
public record PathAction(String source, String name, double expectedSeconds) implements Action {
    public PathAction {
        source = source == null || source.isBlank() ? "path" : source.trim();
        name = name == null || name.isBlank() ? "default" : name.trim();
    }

    public PathAction(String source, String name) {
        this(source, name, Double.NaN);
    }

    public PathAction seconds(double seconds) {
        return new PathAction(source, name, seconds);
    }

    public OptionalDouble expectedDurationSeconds() {
        return Double.isFinite(expectedSeconds) ? OptionalDouble.of(expectedSeconds) : OptionalDouble.empty();
    }

    public String key() {
        return source + ":" + name;
    }

    public boolean samePath(PathAction other) {
        return other != null
                && Objects.equals(source, other.source)
                && Objects.equals(name, other.name);
    }
}
