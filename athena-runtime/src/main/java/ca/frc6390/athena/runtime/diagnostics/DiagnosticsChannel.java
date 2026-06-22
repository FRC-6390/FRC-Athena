package ca.frc6390.athena.runtime.diagnostics;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Named diagnostics channel with bounded event history and live summary fields.
 */
public final class DiagnosticsChannel {
    private final String name;
    private final BoundedEventLog eventLog;
    private final Map<String, String> summary = new LinkedHashMap<>();

    /**
     * Creates a diagnostics channel.
     *
     * @param name channel name
     * @param capacity event capacity
     */
    public DiagnosticsChannel(String name, int capacity) {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("diagnostics channel name must be nonblank");
        }
        this.name = name;
        this.eventLog = new BoundedEventLog(capacity);
    }

    /**
     * Returns channel name.
     *
     * @return channel name
     */
    public String name() {
        return name;
    }

    /**
     * Sets a summary field.
     *
     * @param key field key
     * @param value field value
     * @return this channel
     */
    public DiagnosticsChannel summary(String key, Object value) {
        if (key != null && !key.isBlank()) {
            summary.put(key, String.valueOf(value));
        }
        return this;
    }

    /**
     * Appends an info event.
     *
     * @param message event message
     * @return appended event
     */
    public DiagnosticEvent info(String message) {
        return eventLog.append(DiagnosticLevel.INFO, message);
    }

    /**
     * Appends a warning event.
     *
     * @param message event message
     * @return appended event
     */
    public DiagnosticEvent warn(String message) {
        return eventLog.append(DiagnosticLevel.WARN, message);
    }

    /**
     * Appends an error event.
     *
     * @param message event message
     * @return appended event
     */
    public DiagnosticEvent error(String message) {
        return eventLog.append(DiagnosticLevel.ERROR, message);
    }

    /**
     * Captures summary fields and recent events.
     *
     * @param eventLimit maximum number of events
     * @return diagnostics snapshot
     */
    public DiagnosticsSnapshot snapshot(int eventLimit) {
        return new DiagnosticsSnapshot(name, summary, eventLog.snapshot(eventLimit));
    }

    /**
     * Clears event history while preserving channel identity and summary fields.
     */
    public void clear() {
        eventLog.clear();
    }
}
