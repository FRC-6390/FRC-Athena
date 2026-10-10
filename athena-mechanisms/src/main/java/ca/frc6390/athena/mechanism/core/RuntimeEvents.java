package ca.frc6390.athena.mechanism.core;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

/**
 * Observation channel for runtime events that exploration and diagnostics need but that are not faults.
 *
 * <p>Any number of listeners may be active at once; each receives every event until its subscription is closed.
 * Emitting is cheap when nobody listens: {@link #emit} reads a copy-on-write list and returns when it is empty.
 * Used by {@code ca.frc6390.athena.robot.explore.SimExplorer} and by the replay marker bridge. Emission happens on
 * the robot thread, and listeners must not block.</p>
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
    /** A staging moved to another stage (B9). The message names the stages and the time. */
    public static final String STAGE_CHANGE = "stage.change";
    /** A runtime fault was queued, from a lease exception, a staging condition, or {@code Actions.fault}. */
    public static final String FAULT = "fault";

    private static final List<Subscriber> LISTENERS = new CopyOnWriteArrayList<>();

    private record Subscriber(Consumer<Event> consumer) {
    }

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
     * Adds a listener.
     *
     * @param consumer receives every event emitted until the subscription is closed
     * @return subscription handle
     */
    public static Subscription listen(Consumer<Event> consumer) {
        Subscriber subscriber = new Subscriber(Objects.requireNonNull(consumer, "consumer"));
        LISTENERS.add(subscriber);
        return () -> LISTENERS.remove(subscriber);
    }

    /**
     * Emits an event to every active listener.
     *
     * @param category event category
     * @param message event detail
     */
    public static void emit(String category, String message) {
        if (LISTENERS.isEmpty()) {
            return;
        }
        Event event = new Event(category, message);
        for (Subscriber subscriber : LISTENERS) {
            subscriber.consumer().accept(event);
        }
    }
}
