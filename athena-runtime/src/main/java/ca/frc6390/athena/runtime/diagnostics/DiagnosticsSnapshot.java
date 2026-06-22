package ca.frc6390.athena.runtime.diagnostics;

import java.util.List;
import java.util.Map;

/**
 * Immutable diagnostics channel snapshot.
 *
 * @param channel channel name
 * @param summary summary key-value metadata
 * @param events recent events
 */
public record DiagnosticsSnapshot(String channel, Map<String, String> summary, List<DiagnosticEvent> events) {
    public DiagnosticsSnapshot {
        channel = channel == null || channel.isBlank() ? "diagnostics" : channel;
        summary = Map.copyOf(summary);
        events = List.copyOf(events);
    }

    /**
     * Returns whether this snapshot contains at least one error event.
     *
     * @return true if an error event is present
     */
    public boolean hasErrors() {
        return events.stream().anyMatch(event -> event.level() == DiagnosticLevel.ERROR);
    }
}
