package ca.frc6390.athena.telemetry.networktables;

/**
 * Minimal writer implemented by NetworkTables adapters or tests.
 */
public interface NetworkTableWriter {
    /**
     * Writes a boolean value.
     *
     * @param path normalized NetworkTables path
     * @param value value
     */
    void putBoolean(String path, boolean value);

    /**
     * Writes a numeric value.
     *
     * @param path normalized NetworkTables path
     * @param value value
     */
    void putNumber(String path, double value);

    /**
     * Writes a string value.
     *
     * @param path normalized NetworkTables path
     * @param value value
     */
    void putString(String path, String value);
}
