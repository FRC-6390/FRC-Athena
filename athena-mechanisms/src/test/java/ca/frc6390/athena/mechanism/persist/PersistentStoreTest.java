package ca.frc6390.athena.mechanism.persist;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class PersistentStoreTest {
    @TempDir
    Path directory;

    @Test
    void typedValuesRoundTripThroughTheFile() {
        PersistentStore store = PersistentStore.open(directory);
        store.put("robot/arm/Config/PID/p", 2.5);
        store.put("robot/arm/count", 7L);
        store.put("robot/arm/enabled", true);
        store.put("robot/arm/name", "tab\tand\nnewline\\slash");
        store.put("robot/arm/points", new double[] {1.0, -2.5, 0.0});
        store.put("robot/arm/empty", new double[0]);
        assertTrue(store.flush());

        PersistentStore reopened = PersistentStore.open(directory);
        assertEquals(2.5, reopened.getDouble("robot/arm/Config/PID/p").orElseThrow());
        assertEquals(7L, reopened.get("robot/arm/count").orElseThrow());
        assertEquals(Boolean.TRUE, reopened.get("robot/arm/enabled").orElseThrow());
        assertEquals("tab\tand\nnewline\\slash", reopened.get("robot/arm/name").orElseThrow());
        assertArrayEquals(new double[] {1.0, -2.5, 0.0}, (double[]) reopened.get("robot/arm/points").orElseThrow());
        assertArrayEquals(new double[0], (double[]) reopened.get("robot/arm/empty").orElseThrow());
        assertTrue(reopened.get("robot/arm/missing").isEmpty());
    }

    @Test
    void storedArraysAreCopied() {
        PersistentStore store = PersistentStore.open(directory);
        double[] values = {1.0, 2.0};
        store.put("points", values);
        values[0] = 9.0;
        double[] stored = (double[]) store.get("points").orElseThrow();
        stored[1] = 9.0;
        assertArrayEquals(new double[] {1.0, 2.0}, (double[]) store.get("points").orElseThrow());
    }

    @Test
    void batchedFlushWritesAtMostOncePerSecond() {
        PersistentStore store = PersistentStore.open(directory);
        store.put("a", 1.0);
        assertTrue(store.flushIfDue(10.0), "the first pending change is written immediately");
        assertFalse(store.dirty());

        store.put("a", 2.0);
        assertFalse(store.flushIfDue(10.5), "a second write inside the one second window waits");
        assertTrue(store.dirty());
        assertTrue(store.flushIfDue(11.0));
        assertFalse(store.dirty());
        assertEquals(2.0, PersistentStore.open(directory).getDouble("a").orElseThrow());
    }

    @Test
    void unchangedValuesDoNotDirtyTheStore() {
        PersistentStore store = PersistentStore.open(directory);
        assertTrue(store.put("a", 1.0));
        store.flush();
        assertFalse(store.put("a", 1.0));
        assertFalse(store.dirty());
        assertTrue(store.put("a", 1.5));
        assertTrue(store.dirty());
    }

    @Test
    void writesUseAtomicRenameAndLeaveNoTemporaryFile() throws Exception {
        PersistentStore store = PersistentStore.open(directory);
        store.put("a", 1.0);
        assertTrue(store.flush());
        assertTrue(Files.exists(store.file()));
        assertFalse(Files.exists(directory.resolve(PersistentStore.FILE_NAME + ".tmp")));
        String text = Files.readString(store.file(), StandardCharsets.UTF_8);
        assertTrue(text.contains("a\tD\t1.0"), text);
    }

    @Test
    void retainOnlyDeletesKeysThatNoLongerExist() {
        PersistentStore store = PersistentStore.open(directory);
        store.put("kept", 1.0);
        store.put("renamed/old", 2.0);
        assertEquals(1, store.retainOnly(Set.of("kept")));
        assertEquals(Set.of("kept"), store.keys());
        assertTrue(store.dirty());
    }

    @Test
    void malformedLinesAreSkippedAndCounted() throws Exception {
        Files.writeString(directory.resolve(PersistentStore.FILE_NAME),
                "# header\ngood\tD\t3.5\nbad line without tabs\nbroken\tD\tnot-a-number\nflag\tB\tmaybe\n",
                StandardCharsets.UTF_8);
        PersistentStore store = PersistentStore.open(directory);
        assertEquals(3.5, store.getDouble("good").orElseThrow());
        assertEquals(3, store.skippedLines());
        assertEquals(1, store.size());
    }

    @Test
    void nonFiniteNumbersAreRejected() {
        PersistentStore store = PersistentStore.open(directory);
        assertThrows(IllegalArgumentException.class, () -> store.put("a", Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> store.put("a", new double[] {Double.POSITIVE_INFINITY}));
        assertThrows(IllegalArgumentException.class, () -> store.put(" ", 1.0));
    }
}
