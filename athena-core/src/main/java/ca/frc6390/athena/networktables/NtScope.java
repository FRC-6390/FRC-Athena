package ca.frc6390.athena.networktables;

import com.fasterxml.jackson.core.type.TypeReference;
import java.util.Optional;

/**
 * Scoped Athena NetworkTables API rooted under {@code /Athena}.
 */
public interface NtScope {
    /**
     * Absolute scope path (for example {@code /Athena/Mechanisms/Arm/NetworkTables}).
     */
    String path();

    /**
     * Returns the global Athena root scope ({@code /Athena}).
     */
    NtScope root();

    /**
     * Returns a child scope relative to this scope.
     */
    NtScope scope(String relative);

    /**
     * Unified publish method for primitive, array, enum, and object payloads.
     */
    void put(String key, Object value);

    boolean getBoolean(String key, boolean defaultValue);

    long getInt(String key, long defaultValue);

    double getDouble(String key, double defaultValue);

    String getString(String key, String defaultValue);

    <T> T get(String key, Class<T> type, T defaultValue);

    <T> T get(String key, TypeReference<T> type, T defaultValue);

    <T> Optional<T> tryGet(String key, Class<T> type);

    boolean contains(String key);

    AthenaNTBinding bind(Object target);
}
