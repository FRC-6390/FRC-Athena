package ca.frc6390.athena.mechanism.model;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.Optional;

/** Bounded, oldest-first list of model snapshots, for example one per match. */
public final class ModelHistory {

    /**
     * One saved model.
     *
     * @param timestampSeconds monotonic time when the snapshot was taken
     * @param reason why the snapshot was taken, such as "match" or "disable"
     * @param model the model at that time
     */
    public record Snapshot(double timestampSeconds, String reason, MechanismModel model) {
    }

    private final int maxEntries;
    private final Deque<Snapshot> entries = new ArrayDeque<>();

    public ModelHistory(int maxEntries) {
        if (maxEntries <= 0) {
            throw new IllegalArgumentException("History must keep at least one snapshot.");
        }
        this.maxEntries = maxEntries;
    }

    /** Appends a snapshot, dropping the oldest when the bound is reached. */
    public synchronized void add(Snapshot snapshot) {
        if (entries.size() == maxEntries) {
            entries.removeFirst();
        }
        entries.addLast(snapshot);
    }

    /** Returns snapshots oldest first. */
    public synchronized List<Snapshot> snapshots() {
        return List.copyOf(new ArrayList<>(entries));
    }

    /** Returns the most recent snapshot. */
    public synchronized Optional<Snapshot> latest() {
        return Optional.ofNullable(entries.peekLast());
    }

    public int maxEntries() {
        return maxEntries;
    }
}
