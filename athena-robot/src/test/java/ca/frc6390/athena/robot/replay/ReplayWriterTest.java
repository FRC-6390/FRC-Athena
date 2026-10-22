package ca.frc6390.athena.robot.replay;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.robot.RuntimeCycleSnapshot;
import ca.frc6390.athena.robot.RuntimeFailureReporter;
import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import ca.frc6390.athena.hardware.runtime.HardwareCycleSnapshot;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ReplayWriterTest {
    @TempDir
    Path directory;

    @Test
    void flushHandsOverWithoutWritingOnTheRobotThreadAndTheFileAppearsAfterTheWriterDrains() throws Exception {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        recorder.clock(() -> LocalDateTime.of(2026, 4, 1, 10, 0, 0));
        for (int cycle = 1; cycle <= 2000; cycle++) {
            ReplayRecorderTest.record(recorder, ReplayRecorderTest.snapshot(cycle, cycle * 0.02, 1.0, true), null, List.of());
        }

        long start = System.nanoTime();
        Path target = recorder.flush().orElseThrow();
        long elapsedMicros = (System.nanoTime() - start) / 1_000;

        assertTrue(elapsedMicros < 200_000, "flush took " + elapsedMicros + " us");
        assertTrue(recorder.awaitFlushed(Duration.ofSeconds(10)));
        assertTrue(Files.exists(target));
        assertEquals(target, recorder.lastFile().orElseThrow());
        assertEquals(0, recorder.droppedFlushes());
        try (ReplayReader reader = ReplayReader.open(target)) {
            assertTrue(reader.blockCount() > 1);
        }
    }

    @Test
    void fullQueueDropsTheOldestPendingWriteAndCountsIt(@TempDir Path dir) throws Exception {
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        List<String> reports = new ArrayList<>();
        RuntimeFailureReporter reporter = new RuntimeFailureReporter() {
            @Override public void error(String message, Throwable cause) { reports.add(message); }
            @Override public void warning(String message, Throwable cause) { reports.add(message); }
        };
        ReplayWriter writer = new ReplayWriter(reporter, name -> { }, job -> {
            if (job.reason().equals("first")) {
                started.countDown();
                try {
                    release.await(10, TimeUnit.SECONDS);
                } catch (InterruptedException interrupted) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        try {
            assertTrue(writer.submit(job(dir, "first", "one")));
            assertTrue(started.await(10, TimeUnit.SECONDS));
            // The writer thread holds "first". Two more fill the queue; the next one drops the oldest pending.
            assertTrue(writer.submit(job(dir, "second", "two")));
            assertTrue(writer.submit(job(dir, "third", "three")));
            assertTrue(writer.submit(job(dir, "fourth", "four")));
            assertEquals(1, writer.dropped());

            release.countDown();
            assertTrue(writer.awaitIdle(Duration.ofSeconds(10)));
            assertEquals(3, writer.written());
            assertTrue(Files.exists(dir.resolve("one.athrp")));
            assertFalse(Files.exists(dir.resolve("two.athrp")), "oldest pending write was dropped");
            assertTrue(Files.exists(dir.resolve("three.athrp")));
            assertTrue(Files.exists(dir.resolve("four.athrp")));
            assertTrue(reports.stream().anyMatch(message -> message.contains("dropped 1")), reports::toString);
        } finally {
            release.countDown();
            writer.close();
        }
    }

    @Test
    void byteBudgetDropsOldestPendingSnapshotAndKeepsCurrentAndNewest() throws Exception {
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        ReplayWriter writer = new ReplayWriter(RuntimeFailureReporter.stderr(), name -> { }, job -> {
            if (job.reason().equals("current")) {
                started.countDown();
                try {
                    release.await(5, TimeUnit.SECONDS);
                } catch (InterruptedException interrupted) {
                    Thread.currentThread().interrupt();
                }
            }
        }, 3072);
        try {
            assertTrue(writer.submit(sizedJob(directory, "current", 1024)));
            assertTrue(started.await(2, TimeUnit.SECONDS));
            assertTrue(writer.submit(sizedJob(directory, "old", 1024)));
            assertTrue(writer.submit(sizedJob(directory, "newest", 2048)));
            assertEquals(3072, writer.retainedBytes());
            assertEquals(2, writer.pending());
            assertEquals(1, writer.dropped());
            assertFalse(writer.submit(sizedJob(directory, "oversized", 4096)));
            assertEquals(2, writer.dropped());
            assertEquals(3072, writer.retainedBytes());
            release.countDown();
            assertTrue(writer.awaitIdle(Duration.ofSeconds(5)));
            assertEquals(0, writer.retainedBytes());
            assertTrue(Files.exists(directory.resolve("current.athrp")));
            assertTrue(Files.exists(directory.resolve("newest.athrp")));
            assertFalse(Files.exists(directory.resolve("old.athrp")));
        } finally {
            release.countDown();
            writer.close();
        }
    }

    @Test
    void byteBudgetRejectsNewSnapshotWhenOnlyCurrentWriteUsesTheBudget() throws Exception {
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        ReplayWriter writer = new ReplayWriter(RuntimeFailureReporter.stderr(), name -> { }, job -> {
            started.countDown();
            try {
                release.await(5, TimeUnit.SECONDS);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
            }
        }, 3072);
        try {
            assertTrue(writer.submit(sizedJob(directory, "current", 2048)));
            assertTrue(started.await(2, TimeUnit.SECONDS));
            assertFalse(writer.submit(sizedJob(directory, "refused", 2048)));
            assertEquals(1, writer.dropped());
            assertEquals(2048, writer.retainedBytes());
            release.countDown();
            assertTrue(writer.awaitIdle(Duration.ofSeconds(5)));
            assertFalse(Files.exists(directory.resolve("refused.athrp")));
        } finally {
            release.countDown();
            writer.close();
        }
    }

    @Test
    void recorderSharesItsHistoryBudgetWithAnUnfinishedDiskWrite() throws Exception {
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        List<String> reports = new java.util.concurrent.CopyOnWriteArrayList<>();
        RuntimeFailureReporter reporter = new RuntimeFailureReporter() {
            @Override public void error(String message, Throwable cause) { reports.add(message); }
            @Override public void warning(String message, Throwable cause) { reports.add(message); }
        };
        ReplayWriter writer = new ReplayWriter(reporter, name -> { }, job -> {
            started.countDown();
            try {
                release.await(5, TimeUnit.SECONDS);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
            }
        }, 1536 * 1024);
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of().buffer(512 * 1024), directory, null);
        java.lang.reflect.Field field = ReplayRecorder.class.getDeclaredField("writer");
        field.setAccessible(true);
        field.set(recorder, writer);
        try {
            assertTrue(writer.submit(sizedJob(directory, "slow-disk", 384 * 1024)));
            assertTrue(started.await(2, TimeUnit.SECONDS));
            for (int i = 1; i <= 10000; i++) {
                ReplayRecorderTest.record(recorder, ReplayRecorderTest.snapshot(i, i * 0.02, i * 0.001, true), null, List.of());
            }
            recorder.keyframe();
            assertTrue(recorder.awaitEncoded(Duration.ofSeconds(5))); // Explicit test fence, never a robot hook.
            java.lang.reflect.Field ringField = ReplayRecorder.class.getDeclaredField("buffer");
            ringField.setAccessible(true);
            ReplayBuffer ring = (ReplayBuffer) ringField.get(recorder);
            assertTrue(recorder.recordingEvictions() > 0);
            assertTrue(ring.bytes() + recorder.pendingReplayBytes() <= 512 * 1024,
                    "the ring must subtract the current disk snapshot instead of growing another full history");
            assertFalse(recorder.backfill(9995, 10000).isEmpty(), "the newest live data stays recoverable");
            for (int batch = 0; batch < 6; batch++) {
                for (int i = 10001 + batch * 1000; i <= 11000 + batch * 1000; i++) {
                    ReplayRecorderTest.record(recorder, ReplayRecorderTest.snapshot(i, i * 0.02, i * 0.001, true), null, List.of());
                }
                recorder.flush().orElseThrow();
                assertTrue(recorder.awaitEncoded(Duration.ofSeconds(5)));
                assertTrue(ring.bytes() + recorder.pendingReplayBytes() <= 512 * 1024 + 2 * 64 * 1024,
                        "repeated flush/refill must keep shared bytes bounded, including envelope/newest overhead");
            }
            assertTrue(recorder.droppedFlushes() > 0, "pending flush loss must be counted during a stalled write");
            release.countDown();
            assertTrue(writer.awaitIdle(Duration.ofSeconds(5)));
            assertTrue(reports.stream().anyMatch(message -> message.contains("dropped") || message.contains("evicted")),
                    "memory pressure must be reported rather than silently losing history");
            try (ReplayReader reader = ReplayReader.open(recorder.lastFile().orElseThrow())) {
                ReplayReader.Cursor cursor = reader.cursor();
                long last = -1;
                while (cursor.next()) if (cursor.isFrame()) last = cursor.sequence();
                assertEquals(16000, last, "the newest admitted flush still progresses after disk recovery");
            }
        } finally {
            release.countDown();
            recorder.close();
            assertTrue(writer.awaitIdle(Duration.ofSeconds(5)));
        }
    }

    @Test
    void rejectedDiskFlushKeepsNewestHistoryAvailableForRetry() throws Exception {
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        ReplayWriter writer = new ReplayWriter(RuntimeFailureReporter.stderr(), name -> { }, job -> {
            if (job.reason().equals("current")) {
                started.countDown();
                try {
                    release.await(5, TimeUnit.SECONDS);
                } catch (InterruptedException interrupted) {
                    Thread.currentThread().interrupt();
                }
            }
        }, 64 * 1024);
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of().buffer(256 * 1024), directory, null);
        java.lang.reflect.Field field = ReplayRecorder.class.getDeclaredField("writer");
        field.setAccessible(true);
        field.set(recorder, writer);
        try {
            assertTrue(writer.submit(sizedJob(directory, "current", 64 * 1024)));
            assertTrue(started.await(2, TimeUnit.SECONDS));
            for (int i = 1; i <= 100; i++) {
                ReplayRecorderTest.record(recorder, ReplayRecorderTest.snapshot(i, i * 0.02, i * 0.001, true), null, List.of());
            }
            assertTrue(recorder.flush().isPresent(), "flush reserves a request; disk admission is asynchronous");
            assertTrue(recorder.awaitEncoded(Duration.ofSeconds(5)));
            assertEquals(1, recorder.droppedFlushes());
            assertFalse(recorder.backfill(95, 100).isEmpty(), "refused flush must not discard newest recoverable history");
            release.countDown();
            assertTrue(writer.awaitIdle(Duration.ofSeconds(5)));
            assertTrue(recorder.flush().isPresent());
            assertTrue(recorder.awaitFlushed(Duration.ofSeconds(5)));
            try (ReplayReader reader = ReplayReader.open(recorder.lastFile().orElseThrow())) {
                ReplayReader.Cursor cursor = reader.cursor();
                long last = -1;
                while (cursor.next()) if (cursor.isFrame()) last = cursor.sequence();
                assertEquals(100, last);
            }
        } finally {
            release.countDown();
            recorder.close();
            assertTrue(writer.awaitIdle(Duration.ofSeconds(5)));
        }
    }

    @Test
    void closedWriterRefusesJobsAndFinishesQueuedWork() throws Exception {
        ReplayWriter writer = new ReplayWriter(RuntimeFailureReporter.stderr(), name -> { });
        assertTrue(writer.submit(job(directory, "late", "late")));
        writer.close();
        assertFalse(writer.submit(job(directory, "refused", "refused")));
        assertTrue(writer.awaitIdle(Duration.ofSeconds(10)));
        assertTrue(Files.exists(directory.resolve("late.athrp")));
        assertFalse(Files.exists(directory.resolve("refused.athrp")));
    }

    @Test
    void brownoutMarkerUsesTheE1WKind14() {
        assertEquals(14, ReplayMarkers.BROWNOUT);
    }

    @Test
    void inputAndLocalizationSignalsRoundTripThroughTheFile() throws Exception {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        recorder.clock(() -> LocalDateTime.of(2026, 4, 1, 10, 0, 0));
        RuntimeInputSnapshot inputs = new RuntimeInputSnapshot(
                List.of(new RuntimeInputSnapshot.Joystick(2, true, "Logitech", 3,
                        new double[] {0.25, -0.5}, 0b1011L, new int[] {90, -1})),
                new RuntimeInputSnapshot.DriverStationState(true, true, false, true, true, "blue", 97.5,
                        "2026mrcmp", "QUALIFICATION", 12, "msg"));
        ReplayRecorderTest.record(recorder, withInputs(1, inputs), null, List.of());
        Path file = recorder.flush().orElseThrow();
        assertTrue(recorder.awaitFlushed(Duration.ofSeconds(10)));

        try (ReplayReader reader = ReplayReader.open(file)) {
            ReplayReader.Cursor cursor = reader.cursor();
            assertTrue(cursor.next());
            RuntimeInputSnapshot read = ReplayInputFields.read(cursor);
            RuntimeInputSnapshot.Joystick pad = read.joystick(2);
            assertTrue(pad.connected());
            assertEquals("Logitech", pad.name());
            assertEquals(3, pad.type());
            assertEquals(0.25, pad.axis(0), 1.0e-12);
            assertEquals(-0.5, pad.axis(1), 1.0e-12);
            assertTrue(pad.button(1));
            assertTrue(pad.button(2));
            assertFalse(pad.button(3));
            assertTrue(pad.button(4));
            assertEquals(90, pad.pov(0));
            assertFalse(read.joystick(0).connected());
            RuntimeInputSnapshot.DriverStationState state = read.driverStation();
            assertTrue(state.enabled());
            assertEquals("blue", state.alliance());
            assertEquals(97.5, state.matchTime(), 1.0e-12);
            assertEquals("QUALIFICATION", state.matchType());
            assertEquals(12, state.matchNumber());
            assertEquals("2026mrcmp", state.eventName());
            assertEquals("msg", state.gameMessage());
        }
    }

    @Test
    void seekTimeLandsOnTheSameEventAsALinearScan() throws Exception {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        recorder.clock(() -> LocalDateTime.of(2026, 4, 1, 10, 0, 0));
        for (int cycle = 1; cycle <= 6000; cycle++) {
            ReplayRecorderTest.record(recorder, ReplayRecorderTest.snapshot(cycle, cycle * 0.02, 1.0, true), null, List.of());
        }
        recorder.mark(ReplayMarkers.TELEOP_START, "teleop");
        Path file = ReplayRecorderTest.flushed(recorder);

        try (ReplayReader reader = ReplayReader.open(file)) {
            assertTrue(reader.blockCount() > 3, "file has several blocks");
            long previous = Long.MIN_VALUE;
            for (int block = 0; block < reader.blockCount(); block++) {
                long start = reader.blockStartMicros(block);
                assertTrue(start >= previous);
                previous = start;
            }
            for (double seconds : new double[] {0.0, 0.5, 37.0, 60.01, 99.9, 119.0}) {
                long target = Math.round(seconds * 1_000_000.0);
                long expected = linearScanSequence(reader, target);
                ReplayReader.Cursor cursor = reader.cursor();
                cursor.seekTime(target);
                assertTrue(cursor.next(), "event at or after " + seconds);
                assertTrue(cursor.isFrame(), "seek to " + seconds + " s lands on a frame");
                assertEquals(expected, cursor.sequence(), "seek to " + seconds + " s");
                assertTrue(cursor.timestampMicros() >= target);
            }
            ReplayReader.Cursor past = reader.cursor();
            past.seekTime(Math.round(500.0 * 1_000_000.0));
            assertFalse(past.next(), "nothing at or after 500 s");
        }
    }

    @Test
    void exportCopiesAReplayFileToAFileOrDirectory() throws Exception {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        recorder.clock(() -> LocalDateTime.of(2026, 4, 1, 10, 0, 0));
        ReplayRecorderTest.record(recorder, ReplayRecorderTest.snapshot(1, 0.02, 1.0, true), null, List.of());
        Path file = ReplayRecorderTest.flushed(recorder);
        String name = file.getFileName().toString();
        ReplayFiles files = ReplayFiles.at(directory);

        Path intoDirectory = Files.createDirectories(directory.resolve("usb"));
        Path copied = files.export(name, intoDirectory);
        assertEquals(intoDirectory.resolve(name), copied);
        assertArrayEquals(Files.readAllBytes(file), Files.readAllBytes(copied));

        Path renamed = directory.resolve("exports").resolve("copy.athrp");
        assertEquals(renamed, files.export(name, renamed));
        assertArrayEquals(Files.readAllBytes(file), Files.readAllBytes(renamed));
        assertThrows(() -> files.export("missing-20260101-000000.athrp", intoDirectory));
        assertThrows(() -> files.export("../escape.athrp", intoDirectory));
    }

    private static long linearScanSequence(ReplayReader reader, long target) throws IOException {
        ReplayReader.Cursor cursor = reader.cursor();
        while (cursor.next()) {
            if (cursor.isFrame() && cursor.timestampMicros() >= target) {
                return cursor.sequence();
            }
        }
        return -1L;
    }

    private static void assertThrows(org.junit.jupiter.api.function.Executable executable) {
        org.junit.jupiter.api.Assertions.assertThrows(Exception.class, executable);
    }

    private static ReplayWriter.Job sizedJob(Path directory, String reason, int bytes) {
        return new ReplayWriter.Job(directory.resolve(reason + ".athrp"), Map.of(), new byte[0],
                List.of(new byte[bytes]), reason);
    }

    private static ReplayWriter.Job job(Path dir, String reason, String name) {
        return new ReplayWriter.Job(dir.resolve(name + ".athrp"), Map.of("reason", reason), new byte[0], List.of(),
                reason);
    }

    private static RuntimeCycleSnapshot withInputs(long sequence, RuntimeInputSnapshot inputs) {
        return new RuntimeCycleSnapshot(
                sequence, 0.02 * sequence, 0.02, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, false, false,
                HardwareCycleSnapshot.empty(), List.of(), List.of(), List.of(), inputs);
    }
}
