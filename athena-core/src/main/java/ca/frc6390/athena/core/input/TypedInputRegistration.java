package ca.frc6390.athena.core.input;

import java.util.Map;
import java.util.Objects;

/**
 * Shared helper for typed input section registrations.
 */
public final class TypedInputRegistration {
    private TypedInputRegistration() {}

    public static <V> void put(Map<String, V> target, String key, V value) {
        Objects.requireNonNull(target, "target");
        target.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(value, "supplier"));
    }
}
