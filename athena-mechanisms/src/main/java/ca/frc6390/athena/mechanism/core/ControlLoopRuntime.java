package ca.frc6390.athena.mechanism.core;

/**
 * Runtime state and calculation for a bound control loop.
 */
public interface ControlLoopRuntime {
    /**
     * Resets runtime loop state.
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
}
