package ca.frc6390.athena.telemetry.networktables;

import ca.frc6390.athena.telemetry.TelemetryKey;
import ca.frc6390.athena.telemetry.TelemetrySink;
import ca.frc6390.athena.telemetry.TelemetryValue;

/**
 * Telemetry sink that writes values to NetworkTables-style paths.
 */
public final class NetworkTablesTelemetrySink implements TelemetrySink {
    private final String rootPath;
    private final NetworkTableWriter writer;

    /**
     * Creates a sink rooted at `/Athena`.
     *
     * @param writer table writer
     */
    public NetworkTablesTelemetrySink(NetworkTableWriter writer) {
        this("/Athena", writer);
    }

    /**
     * Creates a sink with a custom root path.
     *
     * @param rootPath root path
     * @param writer table writer
     */
    public NetworkTablesTelemetrySink(String rootPath, NetworkTableWriter writer) {
        this.rootPath = NetworkTablePath.normalize(rootPath);
        this.writer = writer;
    }

    @Override
    public void publish(TelemetryKey key, TelemetryValue value) {
        String path = NetworkTablePath.from(rootPath, key);
        switch (value.type()) {
            case BOOLEAN -> writer.putBoolean(path, (Boolean) value.value());
            case NUMBER -> writer.putNumber(path, (Double) value.value());
            case STRING -> writer.putString(path, (String) value.value());
        }
    }
}
