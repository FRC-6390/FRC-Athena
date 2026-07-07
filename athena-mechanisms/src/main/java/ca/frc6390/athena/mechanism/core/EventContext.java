package ca.frc6390.athena.mechanism.core;

/**
 * Runtime facts used to evaluate lifecycle events.
 *
 * @param nowSeconds current runtime timestamp
 * @param dtSeconds elapsed seconds since the previous loop
 * @param mode current lifecycle mode
 * @param phase current lifecycle phase
 * @param enabled true while the robot is enabled
 * @param simulation true while running simulation code
 */
public record EventContext(
        double nowSeconds,
        double dtSeconds,
        LifecycleMode mode,
        LifecyclePhase phase,
        boolean enabled,
        boolean simulation) {
    public EventContext {
        mode = mode == null ? LifecycleMode.ROBOT : mode;
        phase = phase == null ? LifecyclePhase.PERIODIC : phase;
    }

    /**
     * Creates a lifecycle context with default timing.
     *
     * @param mode lifecycle mode
     * @param phase lifecycle phase
     * @return event context
     */
    public static EventContext lifecycle(LifecycleMode mode, LifecyclePhase phase) {
        boolean enabled = mode == LifecycleMode.AUTONOMOUS || mode == LifecycleMode.TELEOP || mode == LifecycleMode.TEST;
        boolean simulation = mode == LifecycleMode.SIMULATION;
        return new EventContext(0.0, 0.0, mode, phase, enabled, simulation);
    }

    /**
     * Empty/default context for tests and static analysis.
     *
     * @return robot periodic context
     */
    public static EventContext empty() {
        return lifecycle(LifecycleMode.ROBOT, LifecyclePhase.PERIODIC);
    }
}
