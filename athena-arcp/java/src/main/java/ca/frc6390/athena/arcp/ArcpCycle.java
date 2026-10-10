package ca.frc6390.athena.arcp;

import java.util.ArrayList;
import java.util.List;

/**
 * Builder for the values of one robot cycle. Reuse one instance per cycle by calling
 * {@link #clear()} first.
 */
public final class ArcpCycle {

    /** One signal value in a cycle. */
    public record Entry(long signalId, Object value) {
    }

    private final List<Entry> entries = new ArrayList<>();

    /**
     * Adds a value for this cycle.
     *
     * @param signalId dictionary signal id
     * @param value Boolean, Long, Double, String, boolean[], long[], double[], String[] or {@link ArcpStruct}
     * @return this cycle
     */
    public ArcpCycle set(long signalId, Object value) {
        if (value == null) {
            throw new IllegalArgumentException("value for signal " + signalId + " is null");
        }
        entries.add(new Entry(signalId, value));
        return this;
    }

    public ArcpCycle clear() {
        entries.clear();
        return this;
    }

    public List<Entry> entries() {
        return entries;
    }
}
