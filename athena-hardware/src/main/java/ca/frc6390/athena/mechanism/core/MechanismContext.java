package ca.frc6390.athena.mechanism.core;

/**
 * Runtime lifecycle facts for a mechanism state.
 *
 * @param nowSeconds current runtime timestamp
 * @param timeInStateSeconds seconds since the current state became active
 * @param dtSeconds elapsed seconds since the previous update
 * @param enabled true while the robot is enabled
 * @param autonomous true while autonomous mode is active
 * @param simulation true while running in simulation
 */
public record MechanismContext(
        double nowSeconds,
        double timeInStateSeconds,
        double dtSeconds,
        boolean enabled,
        boolean autonomous,
        boolean simulation) {
    /**
     * Empty/default context for tests and static analysis.
     *
     * @return context with zero times and disabled mode flags
     */
    public static MechanismContext empty() {
        return new MechanismContext(0.0, 0.0, 0.0, false, false, false);
    }

    /**
     * Returns true when teleop is active.
     *
     * @return true for enabled non-autonomous operation
     */
    public boolean teleop() {
        return enabled && !autonomous;
    }
}
