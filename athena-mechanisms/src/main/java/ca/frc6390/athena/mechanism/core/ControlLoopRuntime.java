package ca.frc6390.athena.mechanism.core;

/**
 * Runtime Action and calculation for a bound control loop.
 */
public interface ControlLoopRuntime {
    /**
     * Resets runtime loop Action.
     *
     * @param context current context
     */
    default void reset(ControlLoopContext context) {
    }

    /**
     * Calculates the next control output.
     *
     * @param context current context
     * @return control output
     */
    ControlOutput calculate(ControlLoopContext context);

    /**
     * Observes the combined output after Athena applies actuator saturation.
     * Stateful loops can use this to prevent windup caused by other composed loops.
     *
     * @param context current context
     * @param requested combined output before saturation
     * @param applied output after saturation
     */
    default void applied(ControlLoopContext context, Output requested, Output applied) {
    }

    /**
     * Reports whether this loop's gains changed since the previous call, and records the new gains as
     * seen. The applier uses this to make the next output continuous (bumpless transfer, see
     * {@link #bumpTo(double, double)}). Loops without tunable gains return false.
     *
     * @return true when gains changed since the last call
     */
    default boolean consumeGainChange() {
        return false;
    }

    /**
     * Bumpless transfer: re-seeds this loop's integral term so that the combined output, which is
     * {@code currentVolts} with this loop's present contribution included, moves to {@code targetVolts}.
     * Only the integral changes; the error history is kept. Saturation is measured before clamping, so
     * a clamped loop still transfers correctly.
     *
     * @param targetVolts combined output the transfer must preserve (the last applied output)
     * @param currentVolts combined output computed with the new gains
     * @return true when the loop absorbed the transfer (only the first absorbing loop is re-seeded)
     */
    default boolean bumpTo(double targetVolts, double currentVolts) {
        return false;
    }

    /** Returns cached contributions from the most recent calculation without doing new work. */
    default ControlLoopTrace trace() {
        return ControlLoopTrace.ZERO;
    }
}
