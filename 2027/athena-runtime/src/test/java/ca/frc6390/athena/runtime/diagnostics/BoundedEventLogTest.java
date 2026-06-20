package ca.frc6390.athena.runtime.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class BoundedEventLogTest {
    @Test
    void evictsOldestEventsAndKeepsMonotonicSequences() {
        BoundedEventLog log = new BoundedEventLog(2);

        log.append(DiagnosticLevel.INFO, "one");
        log.append(DiagnosticLevel.WARN, "two");
        log.append(DiagnosticLevel.ERROR, "three");

        var events = log.snapshot(10);

        assertEquals(2, events.size());
        assertEquals(1, events.get(0).sequence());
        assertEquals(2, events.get(1).sequence());
        assertEquals("three", events.get(1).message());
    }

    @Test
    void snapshotReturnsNewestLimitInChronologicalOrder() {
        BoundedEventLog log = new BoundedEventLog(5);

        log.append(DiagnosticLevel.INFO, "one");
        log.append(DiagnosticLevel.INFO, "two");
        log.append(DiagnosticLevel.INFO, "three");

        var events = log.snapshot(2);

        assertEquals("two", events.get(0).message());
        assertEquals("three", events.get(1).message());
    }

    @Test
    void requiresPositiveCapacity() {
        assertThrows(IllegalArgumentException.class, () -> new BoundedEventLog(0));
    }
}
