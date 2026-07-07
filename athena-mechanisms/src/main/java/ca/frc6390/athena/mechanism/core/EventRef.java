package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.ActionRef;

/**
 * Declarative event source for hooks.
 */
public interface EventRef {
    /**
     * Returns a human-readable event name.
     *
     * @return event name
     */
    String name();

    /**
     * Returns how source state should become event activity.
     *
     * @return event activation
     */
    EventActivation activation();

    /**
     * Samples the underlying event source.
     *
     * @param context runtime event context
     * @return source active state
     */
    boolean sourceActive(EventContext context);

    /**
     * Evaluates this event using the previous source state.
     *
     * @param context runtime event context
     * @param previousSourceActive previous source state
     * @return true when the event is active for this tick
     */
    default boolean active(EventContext context, boolean previousSourceActive) {
        boolean active = sourceActive(context);
        return switch (activation()) {
            case LEVEL -> active;
            case RISING -> active && !previousSourceActive;
            case FALLING -> !active && previousSourceActive;
            case PULSE -> active;
        };
    }

    /**
     * Returns true for events that should be treated as one-loop pulses.
     *
     * @return true for pulse-like events
     */
    default boolean pulse() {
        return activation() != EventActivation.LEVEL;
    }

    default HookRef onStart(Runnable action) {
        return Hooks.when(this).onStart(action);
    }

    default HookRef onStart(ActionRef action) {
        return Hooks.when(this).onStart(action);
    }

    default HookRef whileActive(Runnable action) {
        return Hooks.when(this).whileActive(action);
    }

    default HookRef whileActive(ActionRef action) {
        return Hooks.when(this).whileActive(action);
    }

    default HookRef onEnd(Runnable action) {
        return Hooks.when(this).onEnd(action);
    }

    default HookRef onEnd(ActionRef action) {
        return Hooks.when(this).onEnd(action);
    }

    default HookRef onInactive(Runnable action) {
        return Hooks.when(this).onInactive(action);
    }

    default HookRef onInactive(ActionRef action) {
        return Hooks.when(this).onInactive(action);
    }

    default HookRef whileInactive(Runnable action) {
        return Hooks.when(this).whileInactive(action);
    }

    default HookRef whileInactive(ActionRef action) {
        return Hooks.when(this).whileInactive(action);
    }
}
