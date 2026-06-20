package ca.frc6390.athena.wpilib.networktables;

import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

import ca.frc6390.athena.telemetry.networktables.NetworkTableWriter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Bridges Athena telemetry publishing to WPILib NetworkTables.
 */
public final class WpilibNetworkTableWriter implements NetworkTableWriter {
    private final WpilibNetworkTableSink sink;

    /**
     * Creates a writer.
     *
     * @param sink table sink
     */
    public WpilibNetworkTableWriter(WpilibNetworkTableSink sink) {
        this.sink = Objects.requireNonNull(sink, "sink");
    }

    /**
     * Creates a writer backed by a real WPILib NetworkTables instance.
     *
     * @param instance NetworkTables instance
     * @return writer
     */
    public static WpilibNetworkTableWriter forInstance(NetworkTableInstance instance) {
        Objects.requireNonNull(instance, "instance");
        return new WpilibNetworkTableWriter(new CachedNetworkTableSink(path -> new RealEntry(instance.getEntry(path))));
    }

    /**
     * Creates a writer backed by the default WPILib NetworkTables instance.
     *
     * @return writer
     */
    public static WpilibNetworkTableWriter forDefaultInstance() {
        return forInstance(NetworkTableInstance.getDefault());
    }

    @Override
    public void putBoolean(String path, boolean value) {
        sink.setBoolean(path, value);
    }

    @Override
    public void putNumber(String path, double value) {
        sink.setDouble(path, value);
    }

    @Override
    public void putString(String path, String value) {
        sink.setString(path, value);
    }

    static WpilibNetworkTableSink cached(EntryLookup entries) {
        return new CachedNetworkTableSink(entries);
    }

    interface EntryLookup {
        CachedEntry entry(String path);
    }

    interface CachedEntry {
        void setBoolean(boolean value);

        void setDouble(double value);

        void setString(String value);
    }

    private static final class CachedNetworkTableSink implements WpilibNetworkTableSink {
        private final EntryLookup lookup;
        private final Map<String, CachedEntry> entries = new ConcurrentHashMap<>();

        private CachedNetworkTableSink(EntryLookup lookup) {
            this.lookup = Objects.requireNonNull(lookup, "lookup");
        }

        @Override
        public void setBoolean(String path, boolean value) {
            entry(path).setBoolean(value);
        }

        @Override
        public void setDouble(String path, double value) {
            entry(path).setDouble(value);
        }

        @Override
        public void setString(String path, String value) {
            entry(path).setString(value);
        }

        private CachedEntry entry(String path) {
            return entries.computeIfAbsent(path, lookup::entry);
        }
    }

    private record RealEntry(NetworkTableEntry entry) implements CachedEntry {
        private RealEntry {
            Objects.requireNonNull(entry, "entry");
        }

        @Override
        public void setBoolean(boolean value) {
            entry.setBoolean(value);
        }

        @Override
        public void setDouble(double value) {
            entry.setDouble(value);
        }

        @Override
        public void setString(String value) {
            entry.setString(value);
        }
    }
}
