package ca.frc6390.athena.runtime.diagnostics;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

/**
 * Fixed-capacity diagnostic event log.
 */
public final class BoundedEventLog {
    private final int capacity;
    private final ArrayDeque<DiagnosticEvent> events = new ArrayDeque<>();
    private long nextSequence;

    /**
     * Creates a bounded log.
     *
     * @param capacity maximum retained events
     */
    public BoundedEventLog(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("capacity must be positive");
        }
        this.capacity = capacity;
    }

    /**
     * Appends an event.
     *
     * @param level severity
     * @param message message
     * @return appended event
     */
    public DiagnosticEvent append(DiagnosticLevel level, String message) {
        DiagnosticEvent event = DiagnosticEvent.create(nextSequence, level, message);
        nextSequence++;
        events.addLast(event);
        while (events.size() > capacity) {
            events.removeFirst();
        }
        return event;
    }

    /**
     * Returns newest retained events up to a limit.
     *
     * @param limit maximum events to return
     * @return newest events in chronological order
     */
    public List<DiagnosticEvent> snapshot(int limit) {
        if (limit <= 0) {
            return List.of();
        }
        List<DiagnosticEvent> copy = new ArrayList<>(events);
        int from = Math.max(0, copy.size() - limit);
        return List.copyOf(copy.subList(from, copy.size()));
    }

    /**
     * Clears retained events.
     */
    public void clear() {
        events.clear();
    }
}
