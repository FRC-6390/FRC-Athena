package ca.frc6390.athena.runtime.diagnostics;

import java.time.Instant;
import java.util.Objects;

/**
 * One structured diagnostic event.
 *
 * @param sequence monotonic channel-local sequence number
 * @param timestamp event timestamp
 * @param level event severity
 * @param message human-readable message
 */
public record DiagnosticEvent(long sequence, Instant timestamp, DiagnosticLevel level, String message) {
    public DiagnosticEvent {
        timestamp = timestamp == null ? Instant.EPOCH : timestamp;
        level = level == null ? DiagnosticLevel.INFO : level;
        message = message == null || message.isBlank() ? "event" : message;
    }

    /**
     * Creates an event using the current clock.
     *
     * @param sequence sequence number
     * @param level severity
     * @param message message
     * @return event
     */
    public static DiagnosticEvent create(long sequence, DiagnosticLevel level, String message) {
        Objects.requireNonNull(level, "level");
        return new DiagnosticEvent(sequence, Instant.now(), level, message);
    }
}
