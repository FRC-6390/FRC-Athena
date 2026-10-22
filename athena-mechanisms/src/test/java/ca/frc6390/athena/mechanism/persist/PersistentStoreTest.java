package ca.frc6390.athena.mechanism.persist;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Set;
import java.io.IOException;
import java.time.Duration;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
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

    @Test
    void blockedSnapshotWriteDoesNotLockReadsMutationsOrClearANewerGeneration() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicInteger writes = new AtomicInteger();
        PersistentStore store = PersistentStore.open(directory, (file, bytes) -> {
            if (writes.incrementAndGet() == 1) awaitWrite(entered, release);
            PersistentStore.writeAtomically(file, bytes);
        });
        var thread = Executors.newSingleThreadExecutor();
        try {
            store.put("value", 1.0);
            var first = thread.submit(store::flush);
            assertTrue(entered.await(2, TimeUnit.SECONDS));
            assertTimeoutPreemptively(Duration.ofMillis(200), () -> {
                assertEquals(1.0, store.getDouble("value").orElseThrow());
                store.put("value", 2.0);
                assertTrue(store.dirty());
                assertEquals("", store.lastError());
            });
            release.countDown();
            assertTrue(first.get(2, TimeUnit.SECONDS));
            assertTrue(store.dirty(), "older completion cannot acknowledge a newer version");
            assertEquals(1.0, PersistentStore.open(directory).getDouble("value").orElseThrow());
            assertTrue(store.flush());
            assertFalse(store.dirty());
            assertEquals(2.0, PersistentStore.open(directory).getDouble("value").orElseThrow());
        } finally {
            release.countDown();
            thread.shutdownNow();
            store.close();
        }
    }

    @Test
    void telemetryPruneAndBothPeriodicModesOnlyQueueCoalescedWrites() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicInteger writes = new AtomicInteger();
        PersistentStore store = PersistentStore.open(directory, (file, bytes) -> {
            if (writes.incrementAndGet() == 1) awaitWrite(entered, release);
            PersistentStore.writeAtomically(file, bytes);
        });
        try {
            store.put("removed", 5.0);
            AtomicLong version = new AtomicLong();
            TelemetryPersistence telemetry = new TelemetryPersistence(store, () -> -1.0);
            telemetry.bindArray("kept", version::get, () -> new double[] {version.get()}, ignored -> { });
            assertTimeoutPreemptively(Duration.ofMillis(200), () -> assertEquals(1, telemetry.pruneUnbound()));
            assertTrue(entered.await(2, TimeUnit.SECONDS));
            assertTimeoutPreemptively(Duration.ofMillis(200), () -> {
                for (int update = 0; update < 100; update++) {
                    version.incrementAndGet();
                    telemetry.periodic(update, LifecycleMode.DISABLED, false);
                    telemetry.periodic(update + 0.01, LifecycleMode.TELEOP, true);
                }
                assertArrayEquals(new double[] {100.0}, (double[]) store.get("kept").orElseThrow());
            });
            release.countDown();
            store.close();
            assertEquals(2, writes.get(), "one blocked version and the coalesced final version");
            assertArrayEquals(new double[] {100.0}, (double[]) PersistentStore.open(directory).get("kept").orElseThrow());
            assertFalse(store.requestFlush());
            assertThrows(IllegalStateException.class, () -> store.put("late", 1.0));
        } finally {
            release.countDown();
            store.close();
        }
    }

    @Test
    void failedBackgroundWriteRetainsDirtyStateAndExplicitRetryPersistsLatestVersion() throws Exception {
        CountDownLatch failed = new CountDownLatch(1);
        AtomicInteger writes = new AtomicInteger();
        PersistentStore store = PersistentStore.open(directory, (file, bytes) -> {
            if (writes.incrementAndGet() == 1) {
                failed.countDown();
                throw new IOException("blocked storage failure");
            }
            PersistentStore.writeAtomically(file, bytes);
        });
        try {
            store.put("value", 1.0);
            assertTrue(store.requestFlush());
            assertTrue(failed.await(2, TimeUnit.SECONDS));
            assertTimeoutPreemptively(Duration.ofSeconds(2), () -> {
                while (store.lastError().isEmpty()) Thread.yield();
            });
            assertTrue(store.dirty());
            store.put("value", 2.0);
            assertTrue(store.flush(), "explicit durable retry bypasses the background backoff");
            assertFalse(store.dirty());
            assertEquals("", store.lastError());
            assertEquals(2.0, PersistentStore.open(directory).getDouble("value").orElseThrow());
        } finally {
            store.close();
        }
    }

    @Test
    void failedBackgroundRequestsBackOffThenRetryTheLatestGeneration() throws Exception {
        AtomicLong clock = new AtomicLong(Long.MIN_VALUE + 5_000_000_000L);
        AtomicInteger writes = new AtomicInteger();
        CountDownLatch retried = new CountDownLatch(1);
        PersistentStore store = PersistentStore.open(directory, (file, bytes) -> {
            if (writes.incrementAndGet() == 1) throw new IOException("temporary failure");
            PersistentStore.writeAtomically(file, bytes);
            retried.countDown();
        }, clock::get);
        try {
            store.put("value", 1.0);
            assertTrue(store.requestFlush());
            assertTimeoutPreemptively(Duration.ofSeconds(2), () -> {
                while (store.lastError().isEmpty() || store.requestFlush()) Thread.yield();
            });
            store.put("value", 2.0);
            assertFalse(store.requestFlush(), "failed storage cannot create a retry storm");
            clock.addAndGet(1_000_000_000L);
            assertTrue(store.requestFlush());
            assertTrue(retried.await(2, TimeUnit.SECONDS));
            assertEquals(2.0, PersistentStore.open(directory).getDouble("value").orElseThrow());
            assertTimeoutPreemptively(Duration.ofSeconds(2), () -> {
                while (store.dirty()) Thread.yield();
            });
            assertEquals("", store.lastError());
            assertEquals(2, writes.get());
        } finally {
            store.close();
        }
    }

    @Test
    void partialChannelWritesAreRetriedUntilTheWholeBufferIsWritten() throws Exception {
        var written = new java.io.ByteArrayOutputStream();
        var channel = new java.nio.channels.WritableByteChannel() {
            @Override public int write(java.nio.ByteBuffer source) {
                int count = Math.min(3, source.remaining());
                for (int index = 0; index < count; index++) written.write(source.get());
                return count;
            }
            @Override public boolean isOpen() { return true; }
            @Override public void close() { }
        };
        byte[] expected = "all bytes survive a short file-channel write".getBytes(StandardCharsets.UTF_8);
        PersistentStore.writeFully(channel, java.nio.ByteBuffer.wrap(expected));
        assertArrayEquals(expected, written.toByteArray());
    }

    private static void awaitWrite(CountDownLatch entered, CountDownLatch release) throws IOException {
        entered.countDown();
        try {
            if (!release.await(5, TimeUnit.SECONDS)) throw new IOException("test storage gate timed out");
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
            throw new IOException(exception);
        }
    }
}
