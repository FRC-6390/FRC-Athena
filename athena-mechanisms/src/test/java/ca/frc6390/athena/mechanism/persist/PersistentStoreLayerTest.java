package ca.frc6390.athena.mechanism.persist;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class PersistentStoreLayerTest {
    @TempDir
    Path directory;

    @Test
    void robotStoreWinsOverDeployedLayerAndLayerIsNeverWritten() {
        PersistentStore store = PersistentStore.open(directory.resolve("store"));
        store.fallback(Map.of("turret/kP", 2.5, "turret/mode", "fast"));
        assertEquals(2.5, store.getDouble("turret/kP").orElseThrow(), 1e-12);
        assertEquals("fast", store.get("turret/mode").orElseThrow());

        store.put("turret/kP", 3.0);
        assertEquals(3.0, store.getDouble("turret/kP").orElseThrow(), 1e-12);
        assertTrue(store.flush());

        PersistentStore reopened = PersistentStore.open(directory.resolve("store"));
        assertEquals(Map.of("turret/kP", 3.0), Map.copyOf(reopenedEntries(reopened)));
        assertFalse(reopened.keys().contains("turret/mode"));
    }

    @Test
    void tuningFilesRoundTripInStoreFormatAndSkipMalformedLines() throws Exception {
        Path file = directory.resolve("tuning");
        Files.writeString(file, "# comment\nturret/kP\tD\t1.25\nbad line\nflag\tB\ttrue\nname\tS\thello\\tworld\n");
        Map<String, Object> read = PersistentStore.readTuning(file);
        assertEquals(1.25, (Double) read.get("turret/kP"), 1e-12);
        assertEquals(Boolean.TRUE, read.get("flag"));
        assertEquals("hello\tworld", read.get("name"));
        assertEquals(3, read.size());

        String rendered = PersistentStore.renderTuning(read);
        Path copy = directory.resolve("copy");
        Files.writeString(copy, rendered);
        assertEquals(read, PersistentStore.readTuning(copy));
        assertTrue(PersistentStore.readTuning(directory.resolve("missing")).isEmpty());
    }

    private static Map<String, Object> reopenedEntries(PersistentStore store) {
        Map<String, Object> out = new java.util.HashMap<>();
        for (String key : store.keys()) {
            out.put(key, store.get(key).orElseThrow());
        }
        return out;
    }
}
