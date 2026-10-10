package ca.frc6390.athena.mechanism.staging;

import ca.frc6390.athena.mechanism.core.Action;

/** The scheduler services a {@link Staging} uses to run stage Actions and observe gating Actions. */
public interface StageHost {
    /**
     * Requests a stage Action through the normal scheduler.
     *
     * @param action action to request
     */
    void request(Action action);

    /**
     * Cancels a stage Action previously requested.
     *
     * @param action action to cancel
     */
    void cancel(Action action);

    /**
     * Returns whether an Action currently has an active lease.
     *
     * @param action action to check
     * @return true while the action runs
     */
    boolean isRunning(Action action);
}
