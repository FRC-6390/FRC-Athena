package ca.frc6390.athena.core.diagnostics;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Thread-safe bounded event timeline with monotonic sequence ids and FPGA timestamps.
 */
public final class BoundedEventLog<E> {
    @FunctionalInterface
    public interface EventFactory<E> {
        E create(long sequence, double timestampSeconds);
    }

    private final ArrayDeque<E> entries = new ArrayDeque<>();
    private final int capacity;
    private long sequence;

    public BoundedEventLog(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("capacity must be > 0");
        }
        this.capacity = capacity;
        this.sequence = 0L;
    }

    public int capacity() {
        return capacity;
    }

    public E append(EventFactory<E> factory) {
        Objects.requireNonNull(factory, "factory");
        synchronized (entries) {
            sequence++;
            E event = factory.create(sequence, Timer.getFPGATimestamp());
            if (event == null) {
                throw new IllegalArgumentException("event factory returned null");
            }
            entries.addLast(event);
            while (entries.size() > capacity) {
                entries.removeFirst();
            }
            return event;
        }
    }

    public List<E> snapshot() {
        synchronized (entries) {
            return List.copyOf(entries);
        }
    }

    public List<E> snapshot(int limit) {
        if (limit <= 0) {
            return List.of();
        }
        synchronized (entries) {
            int size = entries.size();
            if (size == 0) {
                return List.of();
            }
            int capped = Math.min(limit, size);
            if (capped == size) {
                return List.copyOf(entries);
            }
            int skip = size - capped;
            List<E> out = new ArrayList<>(capped);
            int index = 0;
            for (E event : entries) {
                if (index++ < skip) {
                    continue;
                }
                out.add(event);
            }
            return out;
        }
    }

    public int count() {
        synchronized (entries) {
            return entries.size();
        }
    }

    public void clear() {
        synchronized (entries) {
            entries.clear();
        }
    }
}
