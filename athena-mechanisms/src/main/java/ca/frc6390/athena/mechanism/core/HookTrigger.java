package ca.frc6390.athena.mechanism.core;

/**
 * When a hook binding should run relative to event activity.
 */
public enum HookTrigger {
    /**
     * Runs when an event becomes active.
     */
    ON_START,

    /**
     * Runs every tick while an event is active.
     */
    WHILE_ACTIVE,

    /**
     * Runs when an event stops being active.
     */
    ON_END,

    /**
     * Runs when an event becomes inactive.
     */
    ON_INACTIVE,

    /**
     * Runs every tick while an event is inactive.
     */
    WHILE_INACTIVE;

    /**
     * Checks whether this trigger should fire.
     *
     * @param event event declaration
     * @param wasActive previous event activity
     * @param active current event activity
     * @return true when the trigger should run
     */
    public boolean shouldRun(EventRef event, boolean wasActive, boolean active) {
        return switch (this) {
            case ON_START -> active && (!wasActive || event.pulse());
            case WHILE_ACTIVE -> active;
            case ON_END -> !event.pulse() && !active && wasActive;
            case ON_INACTIVE -> !event.pulse() && !active && wasActive;
            case WHILE_INACTIVE -> !event.pulse() && !active;
        };
    }
}
