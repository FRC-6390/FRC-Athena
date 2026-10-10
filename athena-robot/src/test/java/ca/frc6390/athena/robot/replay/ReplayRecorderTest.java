package ca.frc6390.athena.robot.replay;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.HardwareIdentity;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.HardwareCycleSnapshot;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.robot.RuntimeCycleSnapshot;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ReplayRecorderTest {
    static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X44, 5);

    @TempDir
    Path directory;

    @Test
    void roundTripsValuesDeltasKeyframesAndMarkers() throws IOException {
        ReplayRecorder recorder = new ReplayRecorder(
                ReplayPolicy.of().flushOn(), directory, null);
        recorder.clock(() -> LocalDateTime.of(2026, 3, 7, 14, 30, 5));
        recorder.mark(ReplayMarkers.BOOT, "boot");
        for (int cycle = 1; cycle <= 120; cycle++) {
            recorder.record(snapshot(cycle, cycle * 0.02, 1.5 + (cycle % 2) * 0.25, true), null, List.of());
        }
        recorder.mark(ReplayMarkers.MATCH_END, "end", 0);
        Path file = recorder.flush().orElseThrow();

        assertEquals("practice-20260307-143005.athrp", file.getFileName().toString());
        try (ReplayReader reader = ReplayReader.open(file)) {
            assertEquals("athena-replay", reader.metadata().get("format"));
            assertEquals("explicit", reader.metadata().get("reason"));
            ReplayReader.Cursor cursor = reader.cursor();
            int frames = 0;
            int keyframes = 0;
            List<Integer> markerKinds = new java.util.ArrayList<>();
            while (cursor.next()) {
                if (cursor.isFrame()) {
                    frames++;
                    if (cursor.keyframe()) keyframes++;
                    assertEquals(cycle(frames), cursor.sequence());
                    assertEquals(1.5 + (frames % 2) * 0.25, cursor.f64("hardware/motor/intake/positionRotations"), 0.0);
                    assertEquals(LifecycleMode.TELEOP.name(), cursor.string("lifecycle/mode"));
                    assertTrue(cursor.bool("hardware/motor/intake/connected"));
                    assertEquals("", cursor.string("hardware/motor/intake/failure"));
                    assertEquals("neutral", cursor.string("command/intake/mode"));
                    assertEquals(0.0, cursor.f64("command/intake/value"), 0.0);
                    assertTrue(cursor.bool("digital/limit/active") == (frames % 3 == 0));
                } else {
                    markerKinds.add(cursor.markerKind());
                    if (cursor.markerKind() == ReplayMarkers.MATCH_END) {
                        assertEquals("end", cursor.markerMessage());
                    }
                }
            }
            assertEquals(120, frames);
            assertEquals(3, keyframes, "one keyframe at start and one per elapsed second");
            assertEquals(List.of(ReplayMarkers.BOOT, ReplayMarkers.MATCH_END), markerKinds);
            assertTrue(reader.blockCount() >= 1);
        }
    }

    @Test
    void deltaFramesCarryOnlyChangedValuesAndStayCompact() throws IOException {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        for (int cycle = 1; cycle <= 200; cycle++) {
            recorder.record(snapshot(cycle, cycle * 0.02, 1.0, true), null, List.of());
        }
        Path file = recorder.flush().orElseThrow();
        long size = Files.size(file);
        // 200 cycles with ~20 signals each would be far larger than this if every value were repeated.
        assertTrue(size < 40_000, "size " + size);
        try (ReplayReader reader = ReplayReader.open(file)) {
            ReplayReader.Cursor cursor = reader.cursor();
            int keyframes = 0;
            int frames = 0;
            while (cursor.next()) {
                if (cursor.isFrame()) {
                    frames++;
                    if (cursor.keyframe()) keyframes++;
                }
            }
            assertEquals(200, frames);
            assertEquals(4, keyframes);
        }
    }

    @Test
    void forcedKeyframeAndBlockSeekingDecodeFromTheBlockStart() throws IOException {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        // 2000 cycles with varying values: enough data for several 64 KiB blocks.
        for (int cycle = 1; cycle <= 2000; cycle++) {
            recorder.record(snapshot(cycle, cycle * 0.02, cycle * 0.001, true), null, List.of());
            if (cycle == 1000) recorder.keyframe();
        }
        Path file = recorder.flush().orElseThrow();
        try (ReplayReader reader = ReplayReader.open(file)) {
            assertTrue(reader.blockCount() > 1, "blocks: " + reader.blockCount());
            int target = reader.blockCount() - 1;
            ReplayReader.Cursor cursor = reader.cursor();
            cursor.seekBlock(target);
            assertTrue(cursor.next());
            assertTrue(cursor.isFrame());
            assertTrue(cursor.keyframe(), "first frame of a block is a keyframe");
            long sequence = cursor.sequence();
            assertTrue(sequence > 1 && sequence <= 2000);
            assertEquals(sequence * 0.001, cursor.f64("hardware/motor/intake/velocityRotationsPerSecond"), 0.0);
            assertEquals(sequence * 0.02, cursor.timestampMicros() / 1_000_000.0, 1e-6);
        }
    }

    @Test
    void rejectsFilesThatAreNotReplays() throws IOException {
        Path bogus = directory.resolve("bogus.athrp");
        Files.write(bogus, new byte[] {'N', 'O', 'P', 'E', 0, 1, 0});
        assertThrows(IOException.class, () -> ReplayReader.open(bogus));
    }

    @Test
    void matchIdentificationFlushesPracticeAndNamesTheNextFileAfterTheMatch() throws IOException {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        recorder.clock(() -> LocalDateTime.of(2026, 3, 7, 9, 0, 0));
        recorder.record(snapshot(1, 0.02, 1.0, false), null, List.of());
        recorder.matchIdentified(new ReplayMatch("2026mrcmp", "Qualification", 12));
        recorder.record(snapshot(2, 0.04, 1.0, true), null, List.of());
        recorder.clock(() -> LocalDateTime.of(2026, 3, 7, 9, 5, 0));
        recorder.matchEnd();

        List<String> names = ReplayFiles.at(directory).list().stream().map(ReplayFiles.FileInfo::name).sorted().toList();
        assertEquals(List.of("2026mrcmp-q12-20260307-090500.athrp", "practice-20260307-090000.athrp"), names);
        ReplayFiles.FileInfo match = ReplayFiles.at(directory).list().stream()
                .filter(info -> info.event().equals("2026mrcmp")).findFirst().orElseThrow();
        assertEquals(12, match.matchNumber());
        assertEquals("q", match.matchType());
        assertFalse(match.practice());
    }

    @Test
    void disableFlushesOnlyAfterAnEnabledPeriodThatWasNotFlushed() throws IOException {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        recorder.clock(() -> LocalDateTime.of(2026, 1, 1, 0, 0, 0));
        recorder.record(snapshot(1, 0.02, 1.0, false), null, List.of());
        recorder.disabled();
        assertEquals(0, ReplayFiles.at(directory).list().size(), "no enabled period, no flush");

        recorder.enabledPeriod(ReplayMarkers.TELEOP_START, "teleop");
        recorder.record(snapshot(2, 0.04, 1.0, true), null, List.of());
        recorder.disabled();
        assertEquals(1, ReplayFiles.at(directory).list().size());

        recorder.record(snapshot(3, 0.06, 1.0, false), null, List.of());
        recorder.brownoutRecovered();
        assertEquals(2, ReplayFiles.at(directory).list().size());
    }

    @Test
    void retentionDeletesPracticeBeforeMatchesAndKeepsTheNewestFile() throws IOException {
        Files.createDirectories(directory);
        writeFile("2026mrcmp-q1-20260101-100000.athrp", 1000);
        writeFile("practice-20260101-090000.athrp", 1000);
        writeFile("practice-20260101-110000.athrp", 1000);
        writeFile("2026mrcmp-q2-20260101-120000.athrp", 1000);

        ReplayFiles files = ReplayFiles.at(directory);
        List<String> deleted = files.enforce(
                Retention.maxTotalBytes(2500).keepMatchesOverPractice(), "practice-20260101-110000.athrp");
        assertEquals(List.of("practice-20260101-090000.athrp", "2026mrcmp-q1-20260101-100000.athrp"), deleted);

        deleted = files.enforce(Retention.maxTotalBytes(1500).keepMatchesOverPractice(),
                "2026mrcmp-q2-20260101-120000.athrp");
        assertEquals(List.of("practice-20260101-110000.athrp"), deleted);
        assertEquals(List.of("2026mrcmp-q2-20260101-120000.athrp"),
                files.list().stream().map(ReplayFiles.FileInfo::name).toList());
    }

    @Test
    void minimumFreeDiskRuleRemovesFilesButNeverTheNewest() throws IOException {
        Files.createDirectories(directory);
        writeFile("practice-20260101-090000.athrp", 10);
        writeFile("practice-20260101-100000.athrp", 10);
        List<String> deleted = ReplayFiles.at(directory).enforce(
                Retention.unlimited().minimumFreeDiskBytes(Long.MAX_VALUE), "practice-20260101-100000.athrp");
        assertEquals(List.of("practice-20260101-090000.athrp"), deleted);
        assertTrue(Files.exists(directory.resolve("practice-20260101-100000.athrp")));
    }

    @Test
    void storedFilesRejectPathTraversalAndListOtherCategories() throws IOException {
        Files.createDirectories(directory);
        writeFile("values.json", 5);
        StoredFiles values = StoredFiles.of(directory, ".json");
        assertEquals(List.of("values.json"), values.list().stream().map(StoredFiles.Entry::name).toList());
        assertThrows(IllegalArgumentException.class, () -> values.resolve("../escape.json"));
        assertTrue(values.delete("values.json"));
        assertFalse(values.delete("values.json"));
    }

    @Test
    void sinkReceivesLiveRecordsAndBackfillReturnsBufferedGroups() {
        List<Long> frames = new java.util.ArrayList<>();
        List<byte[]> backfilled = new java.util.ArrayList<>();
        ReplayStreamSink sink = new ReplayStreamSink() {
            @Override public void publishDictionary(byte[] records) { assertTrue(records.length > 0); }
            @Override public void publishFrame(long sequence, byte[] records) { frames.add(sequence); }
            @Override public void publishMarker(long sequence, byte[] record) { }
            @Override public void attach(ReplayBackfill backfill) { backfilled.addAll(backfill.records(1, 3)); }
        };
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of().stream(true), directory, null);
        recorder.streamSink(sink);
        assertTrue(recorder.streaming());
        for (int cycle = 1; cycle <= 5; cycle++) {
            recorder.record(snapshot(cycle, cycle * 0.02, 1.0, true), null, List.of());
        }
        assertEquals(List.of(1L, 2L, 3L, 4L, 5L), frames);
        assertTrue(backfilled.isEmpty(), "nothing buffered at attach time");
        assertEquals(1, recorder.backfill(1, 5).size(), "open segment is backfillable");
        recorder.streamSink(null);
        assertFalse(recorder.streaming());
    }

    @Test
    void streamingIsOffUntilPolicyAndSinkAreBothPresent() {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        recorder.streamSink(new ReplayStreamSink() {
            @Override public void publishDictionary(byte[] records) { throw new AssertionError(); }
            @Override public void publishFrame(long sequence, byte[] records) { throw new AssertionError(); }
            @Override public void publishMarker(long sequence, byte[] record) { throw new AssertionError(); }
        });
        assertFalse(recorder.streaming());
        recorder.record(snapshot(1, 0.02, 1.0, true), null, List.of());
    }

    @Test
    void policyDefaultsAndBuilders() {
        ReplayPolicy defaults = ReplayPolicy.of();
        assertFalse(defaults.streamEnabled());
        assertEquals(48L * 1024 * 1024, defaults.bufferBytes());
        assertTrue(defaults.flushes(ReplayPolicy.Flush.MATCH_END));
        assertTrue(defaults.flushes(ReplayPolicy.Flush.DISABLE_AFTER_MATCH));
        assertTrue(defaults.flushes(ReplayPolicy.Flush.BROWNOUT_RECOVERY));
        assertEquals(60, defaults.retention().maxFiles());
        ReplayPolicy custom = defaults.stream(true).bufferMegabytes(8).flushOn(ReplayPolicy.Flush.MATCH_END)
                .retention(Retention.maxTotalMegabytes(400).maxFiles(60).keepMatchesOverPractice()
                        .minimumFreeDiskMegabytes(150));
        assertTrue(custom.streamEnabled());
        assertEquals(8L * 1024 * 1024, custom.bufferBytes());
        assertFalse(custom.flushes(ReplayPolicy.Flush.BROWNOUT_RECOVERY));
        assertEquals(400L * 1024 * 1024, custom.retention().maxTotalBytes());
        assertEquals(Optional.empty(), defaults.directory());
    }

    @Test
    void markersReachTheActiveRecorderOnly() throws IOException {
        ReplayRecorder recorder = new ReplayRecorder(ReplayPolicy.of(), directory, null);
        ReplayMarkers.activate(recorder);
        try {
            ReplayMarkers.mark(ReplayMarkers.FAULT, "motor disconnected");
            ReplayMarkers.keyframe();
            recorder.record(snapshot(1, 0.02, 1.0, true), null, List.of());
            Path file = recorder.flush().orElseThrow();
            try (ReplayReader reader = ReplayReader.open(file)) {
                ReplayReader.Cursor cursor = reader.cursor();
                assertTrue(cursor.next());
                assertFalse(cursor.isFrame());
                assertEquals(ReplayMarkers.FAULT, cursor.markerKind());
                assertEquals("motor disconnected", cursor.markerMessage());
            }
        } finally {
            ReplayMarkers.activate(null);
        }
    }

    private static long cycle(int frameCount) {
        return frameCount;
    }

    private void writeFile(String name, int bytes) throws IOException {
        Files.write(directory.resolve(name), new byte[bytes]);
    }

    static RuntimeCycleSnapshot snapshot(long sequence, double timestamp, double velocity, boolean enabled) {
        HardwareIdentity identity = HardwareIdentity.motor(MOTOR);
        HardwareCycleSnapshot.MotorInput motor = new HardwareCycleSnapshot.MotorInput(
                "intake", 1.5 + (sequence % 2) * 0.25, velocity, 6.0, 4.0, 8.0, true, "");
        HardwareCycleSnapshot hardware = new HardwareCycleSnapshot(
                sequence, (long) (timestamp * 1e9), Map.of(identity, motor), Map.of(), Map.of());
        return new RuntimeCycleSnapshot(
                sequence, timestamp, 0.02, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, enabled, false, false,
                hardware,
                List.of(new RuntimeCycleSnapshot.DigitalInput("limit", 0, sequence % 3 == 0, sequence % 3 == 0)),
                List.of(), List.of());
    }

}
