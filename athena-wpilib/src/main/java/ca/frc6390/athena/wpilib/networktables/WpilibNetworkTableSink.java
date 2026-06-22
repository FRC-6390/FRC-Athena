package ca.frc6390.athena.wpilib.networktables;

/**
 * Minimal table sink implemented by a real WPILib NetworkTables adapter.
 */
public interface WpilibNetworkTableSink {
    /**
     * Writes a boolean value.
     *
     * @param path normalized path
     * @param value value
     */
    void setBoolean(String path, boolean value);

    /**
     * Writes a numeric value.
     *
     * @param path normalized path
     * @param value value
     */
    void setDouble(String path, double value);

    /**
     * Writes a string value.
     *
     * @param path normalized path
     * @param value value
     */
    void setString(String path, String value);
}
