package ca.frc6390.athena.telemetry.networktables;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * In-memory NetworkTables writer for tests and examples.
 */
public final class InMemoryNetworkTableWriter implements NetworkTableWriter {
    private final Map<String, Object> values = new LinkedHashMap<>();

    @Override
    public void putBoolean(String path, boolean value) {
        values.put(path, value);
    }

    @Override
    public void putNumber(String path, double value) {
        values.put(path, value);
    }

    @Override
    public void putString(String path, String value) {
        values.put(path, value);
    }

    /**
     * Returns published values.
     *
     * @return immutable published values
     */
    public Map<String, Object> values() {
        return Map.copyOf(values);
    }
}
