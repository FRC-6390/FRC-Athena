package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.EventContext;
import java.time.Duration;
import java.util.function.IntSupplier;

/** A shared recognizer for assigning different actions to different click counts. */
public final class ClickSequence {
    private static final Duration DEFAULT_MAX_PRESS = Duration.ofMillis(500);

    private final ButtonSignal source;
    private final Tracker tracker;

    ClickSequence(ButtonSignal source, Duration window) {
        this.source = source;
        tracker = new Tracker(
                source,
                ControlSignal.seconds(window, "window"),
                ControlSignal.seconds(DEFAULT_MAX_PRESS, "maximumPress"));
    }

    /** Emits after a completed sequence containing exactly {@code count} clicks. */
    public ControlSignal exactly(int count) {
        requirePositive(count, "count");
        return source.pulse("clicks[" + count + "]", context -> tracker.count(context) == count);
    }

    /** Emits after a completed sequence containing at least {@code count} clicks. */
    public ControlSignal atLeast(int count) {
        requirePositive(count, "count");
        return source.pulse("clicks[" + count + "+]", context -> tracker.count(context) >= count);
    }

    /** Emits after a completed sequence whose count is inside the inclusive range. */
    public ControlSignal between(int minimum, int maximum) {
        requirePositive(minimum, "minimum");
        if (maximum < minimum) {
            throw new IllegalArgumentException("maximum must be greater than or equal to minimum");
        }
        return source.pulse(
                "clicks[" + minimum + ".." + maximum + "]",
                context -> {
                    int count = tracker.count(context);
                    return count >= minimum && count <= maximum;
                });
    }

    /** Returns the count emitted on the current tick, or zero when none completed. */
    public IntSupplier count() {
        return tracker::lastCount;
    }

    /** Returns whether a click sequence is currently waiting for another click. */
    public boolean pending() {
        return tracker.pending();
    }

    private static void requirePositive(int value, String parameter) {
        if (value < 1) {
            throw new IllegalArgumentException(parameter + " must be at least 1");
        }
    }

    private static final class Tracker {
        private final ButtonSignal source;
        private final double windowSeconds;
        private final double maximumPressSeconds;
        private EventContext lastContext;
        private boolean previousPressed;
        private double pressedAt;
        private double lastReleasedAt;
        private int pendingCount;
        private int emittedCount;

        private Tracker(ButtonSignal source, double windowSeconds, double maximumPressSeconds) {
            this.source = source;
            this.windowSeconds = windowSeconds;
            this.maximumPressSeconds = maximumPressSeconds;
        }

        private int count(EventContext context) {
            update(context);
            return emittedCount;
        }

        private int lastCount() {
            return emittedCount;
        }

        private boolean pending() {
            return pendingCount > 0;
        }

        private void update(EventContext context) {
            if (lastContext == context) {
                return;
            }
            emittedCount = 0;
            boolean pressed = source.sample(context);
            double now = context.nowSeconds();
            if (pressed && !previousPressed) {
                pressedAt = now;
            } else if (!pressed && previousPressed) {
                if (now - pressedAt <= maximumPressSeconds) {
                    pendingCount++;
                    lastReleasedAt = now;
                } else {
                    pendingCount = 0;
                }
            }
            if (!pressed && pendingCount > 0 && now - lastReleasedAt >= windowSeconds) {
                emittedCount = pendingCount;
                pendingCount = 0;
            }
            previousPressed = pressed;
            lastContext = context;
        }
    }
}
