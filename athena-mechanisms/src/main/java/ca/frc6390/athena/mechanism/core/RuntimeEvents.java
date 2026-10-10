package ca.frc6390.athena.mechanism.core;

import java.util.Objects;
import java.util.function.Consumer;

/**
 * Observation channel for runtime events that exploration and diagnostics need but that are not faults.
 *
 * <p>Emitting is free when nobody listens: {@link #emit} is a volatile read and a null check. Listeners are
 * installed with {@link #listen(Consumer)} and removed by closing the returned handle. Only one listener is
 * active at a time; a new listener shadows the previous one until it is closed. The channel is used by
 * {@code ca.frc6390.athena.robot.explore.SimExplorer}. Emission happens on the robot thread.</p>
 */
public final class RuntimeEvents {
    /** An {@code Actions.timeout} or {@code atMost} bound expired before its Action completed. */
    public static final String TIMEOUT = "timeout";
    /** A path finished while some of its markers had not triggered. */
    public static final String PATH_MARKERS_PENDING = "path.markersPending";
    /**
     * An effort limit clamped a command. Reserved for the effort subsystem (B5): it should call
     * {@link #emit} with this category at the point where the ceiling is applied.
     */
    public static final String EFFORT_CEILING = "effort.ceiling";

    private static volatile Consumer<Event> listener;

    private RuntimeEvents() {
    }

    /**
     * One observed event.
     *
     * @param category one of the category constants of this class
     * @param message human-readable detail
     */
    public record Event(String category, String message) {
        public Event {
            Objects.requireNonNull(category, "category");
            message = message == null ? "" : message;
        }
    }

    /** Handle returned by {@link #listen}; closing it restores the previous listener. */
    public interface Subscription extends AutoCloseable {
        @Override
        void close();
    }

    /**
     * Installs a listener.
     *
     * @param consumer receives every event emitted until the subscription is closed
     * @return subscription handle
     */
    public static Subscription listen(Consumer<Event> consumer) {
        Consumer<Event> previous = listener;
        Consumer<Event> installed = Objects.requireNonNull(consumer, "consumer");
        listener = installed;
        return () -> {
            if (listener == installed) {
                listener = previous;
            }
        };
    }

    /**
     * Emits an event to the installed listener, if any.
     *
     * @param category event category
     * @param message event detail
     */
    public static void emit(String category, String message) {
        Consumer<Event> current = listener;
        if (current != null) {
            current.accept(new Event(category, message));
        }
    }
}
