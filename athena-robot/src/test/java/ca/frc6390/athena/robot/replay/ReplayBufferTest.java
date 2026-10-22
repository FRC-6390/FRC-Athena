package ca.frc6390.athena.robot.replay;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class ReplayBufferTest {
    @Test
    void selectionAccountsForLengthPrefixesBeforeCopyingAndDoesNotSkipGroups() {
        ReplayBuffer buffer = new ReplayBuffer(1000);
        buffer.add(segment(1, new byte[127]));
        buffer.add(segment(2, new byte[128]));
        buffer.add(segment(3, new byte[1]));
        assertEquals(128, ReplayBuffer.encodedSize(127));
        assertEquals(130, ReplayBuffer.encodedSize(128));
        List<byte[]> first = buffer.backfill(1, 3, 257);
        assertEquals(1, first.size(), "second group misses the budget by one byte; third must not jump the gap");
        assertEquals(127, first.get(0).length);
        assertEquals(2, buffer.backfill(1, 3, 258).size());
        assertEquals(3, buffer.backfill(1, 3, 260).size());
        assertThrows(IllegalArgumentException.class, () -> buffer.backfill(2, 2, 129));
        assertTrue(buffer.backfill(5, 6, 1).isEmpty());
    }

    @Test
    void selectedCopiesSurviveEvictionAndDoNotExposeRecordedBytes() {
        ReplayBuffer buffer = new ReplayBuffer(4);
        byte[] bytes = {1, 2, 3};
        buffer.add(segment(1, bytes));
        List<byte[]> copy = buffer.backfill(1, 1, 4);
        copy.get(0)[0] = 99;
        assertEquals(1, buffer.backfill(1, 1, 4).get(0)[0]);
        ReplayBuffer.Selection selection = buffer.select(1, 1, 4);
        buffer.add(segment(2, new byte[3]));
        assertEquals(1, selection.segments().get(0).records()[0]);
        assertTrue(buffer.backfill(1, 1, 4).isEmpty());
    }

    @Test
    void fullDefaultHistoryBackfillsWithin96MiBHeap() throws Exception {
        String tests = Path.of(ReplayBackfillHeapProbe.class.getProtectionDomain().getCodeSource()
                .getLocation().toURI()).toString();
        String main = Path.of(ReplayBuffer.class.getProtectionDomain().getCodeSource()
                .getLocation().toURI()).toString();
        Process process = new ProcessBuilder(Path.of(System.getProperty("java.home"), "bin", "java").toString(),
                "-Xms16m", "-Xmx96m", "-cp", tests + File.pathSeparator + main,
                ReplayBackfillHeapProbe.class.getName()).redirectErrorStream(true).start();
        try {
            assertTrue(process.waitFor(15, TimeUnit.SECONDS), "constrained-heap backfill probe did not finish");
            String output = new String(process.getInputStream().readAllBytes(), java.nio.charset.StandardCharsets.UTF_8);
            assertEquals(0, process.exitValue(), output);
            assertTrue(output.contains("history=50331648"), output);
            assertTrue(output.contains("boundedBytes=983085"), output);
        } finally {
            process.destroyForcibly();
        }
    }

    @Test
    void held48MiBDiskSnapshotAndRefillingRingStayWithin96MiBHeap() throws Exception {
        String tests = Path.of(ReplayBackfillHeapProbe.class.getProtectionDomain().getCodeSource()
                .getLocation().toURI()).toString();
        String main = Path.of(ReplayBuffer.class.getProtectionDomain().getCodeSource()
                .getLocation().toURI()).toString();
        String runtime = Path.of(ca.frc6390.athena.runtime.wire.WireWriter.class.getProtectionDomain().getCodeSource()
                .getLocation().toURI()).toString();
        Process process = new ProcessBuilder(Path.of(System.getProperty("java.home"), "bin", "java").toString(),
                "-Xms16m", "-Xmx96m", "-cp", String.join(File.pathSeparator, tests, main, runtime),
                ReplayBackfillHeapProbe.class.getName(), "writer").redirectErrorStream(true).start();
        try {
            assertTrue(process.waitFor(20, TimeUnit.SECONDS), "slow-disk heap probe did not finish");
            String output = new String(process.getInputStream().readAllBytes(), java.nio.charset.StandardCharsets.UTF_8);
            assertEquals(0, process.exitValue(), output);
            assertTrue(output.contains("writerBytes=50331648 liveBytes=65536 evictions=1023"), output);
        } finally {
            process.destroyForcibly();
        }
    }

    private static ReplayBuffer.Segment segment(int sequence, byte[] bytes) {
        return new ReplayBuffer.Segment(bytes, sequence, sequence, sequence * 0.02, sequence * 0.02);
    }
}
