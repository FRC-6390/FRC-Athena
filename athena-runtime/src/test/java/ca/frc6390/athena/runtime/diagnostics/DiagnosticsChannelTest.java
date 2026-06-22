package ca.frc6390.athena.runtime.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DiagnosticsChannelTest {
    @Test
    void snapshotIncludesSummaryAndRecentEvents() {
        DiagnosticsChannel channel = new DiagnosticsChannel("shooter", 4)
                .summary("mode", "speaker")
                .summary("targetRpm", 4600);

        channel.info("enabled");
        channel.warn("slow to target");
        channel.error("sensor disconnected");

        DiagnosticsSnapshot snapshot = channel.snapshot(2);

        assertEquals("shooter", snapshot.channel());
        assertEquals("speaker", snapshot.summary().get("mode"));
        assertEquals("4600", snapshot.summary().get("targetRpm"));
        assertEquals(2, snapshot.events().size());
        assertEquals(DiagnosticLevel.WARN, snapshot.events().get(0).level());
        assertTrue(snapshot.hasErrors());
    }

    @Test
    void clearRemovesEventsButKeepsSummary() {
        DiagnosticsChannel channel = new DiagnosticsChannel("drive", 2)
                .summary("state", "ready");
        channel.info("booted");

        channel.clear();

        DiagnosticsSnapshot snapshot = channel.snapshot(10);
        assertEquals("ready", snapshot.summary().get("state"));
        assertTrue(snapshot.events().isEmpty());
    }

    @Test
    void requiresNonblankName() {
        assertThrows(IllegalArgumentException.class, () -> new DiagnosticsChannel(" ", 2));
    }
}
