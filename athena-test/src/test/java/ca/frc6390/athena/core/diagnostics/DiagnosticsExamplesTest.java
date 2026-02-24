package ca.frc6390.athena.core.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.DiagnosticsExamples;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

final class DiagnosticsExamplesTest {

    @Test
    void boundedEventLogRespectsCapacityAndSequence() {
        BoundedEventLog<String> log = new BoundedEventLog<>(2);
        log.append((seq, ts) -> "e" + seq);
        log.append((seq, ts) -> "e" + seq);
        log.append((seq, ts) -> "e" + seq);

        assertEquals(List.of("e2", "e3"), log.snapshot());
        assertEquals(List.of("e3"), log.snapshot(1));
    }

    @Test
    void diagnosticsExamplePublishesFieldsAndEvents() {
        DiagnosticsChannel channel = DiagnosticsExamples.createSubsystemChannel("drivetrain", 4);
        DiagnosticsExamples.publishLoopTiming(channel, 20.0, 3.1);

        assertEquals(1, channel.eventCount());
        DiagnosticsChannel.Event event = channel.events(5).get(0);
        assertEquals("WARN", event.level());
        assertTrue(event.line().contains("timing"));

        Map<String, Object> snapshot = DiagnosticsExamples.snapshot(channel, 5);
        assertEquals("robot/drivetrain", snapshot.get("name"));
        assertEquals(4, snapshot.get("capacity"));
    }

    @Test
    void diagnosticsClearDropsEvents() {
        DiagnosticsChannel channel = DiagnosticsExamples.createSubsystemChannel("arm", 2);
        channel.info("status", "ready");
        assertEquals(1, channel.eventCount());

        channel.clear();
        assertEquals(0, channel.eventCount());
    }
}
