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
}
