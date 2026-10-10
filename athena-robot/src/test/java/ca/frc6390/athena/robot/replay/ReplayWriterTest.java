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
            recorder.record(ReplayRecorderTest.snapshot(cycle, cycle * 0.02, 1.0, true), null, List.of());
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
        recorder.record(withInputs(1, inputs), null, List.of());
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
            recorder.record(ReplayRecorderTest.snapshot(cycle, cycle * 0.02, 1.0, true), null, List.of());
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
        recorder.record(ReplayRecorderTest.snapshot(1, 0.02, 1.0, true), null, List.of());
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
