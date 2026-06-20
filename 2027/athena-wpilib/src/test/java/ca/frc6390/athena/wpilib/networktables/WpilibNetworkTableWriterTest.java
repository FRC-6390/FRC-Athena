package ca.frc6390.athena.wpilib.networktables;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

class WpilibNetworkTableWriterTest {
    @Test
    void cachedSinkReusesEntriesByPath() {
        var lookup = new RecordingLookup();
        var sink = WpilibNetworkTableWriter.cached(lookup);

        sink.setBoolean("/Athena/enabled", true);
        sink.setDouble("/Athena/enabled", 1.0);
        sink.setString("/Athena/mode", "auto");
        sink.setString("/Athena/mode", "teleop");

        assertEquals(1, lookup.lookupCounts.get("/Athena/enabled"));
        assertEquals(1, lookup.lookupCounts.get("/Athena/mode"));
        assertEquals(1.0, lookup.entries.get("/Athena/enabled").lastValue);
        assertEquals("teleop", lookup.entries.get("/Athena/mode").lastValue);
    }

    private static final class RecordingLookup implements WpilibNetworkTableWriter.EntryLookup {
        private final Map<String, Integer> lookupCounts = new LinkedHashMap<>();
        private final Map<String, RecordingEntry> entries = new LinkedHashMap<>();

        @Override
        public WpilibNetworkTableWriter.CachedEntry entry(String path) {
            lookupCounts.merge(path, 1, Integer::sum);
            var entry = new RecordingEntry();
            entries.put(path, entry);
            return entry;
        }
    }

    private static final class RecordingEntry implements WpilibNetworkTableWriter.CachedEntry {
        private Object lastValue;

        @Override
        public void setBoolean(boolean value) {
            lastValue = value;
        }

        @Override
        public void setDouble(double value) {
            lastValue = value;
        }

        @Override
        public void setString(String value) {
            lastValue = value;
        }
    }
}
