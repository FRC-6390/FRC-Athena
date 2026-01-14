package ca.frc6390.athena.mechanisms;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Marker for mechanisms that own a state machine with a Double setpoint.
 * Implemented by all stateful mechanism variants (generic, arm, elevator, turret, etc.).
 *
 * @param <E> state enum type
 */
public interface StatefulLike<E extends Enum<E> & StateMachine.SetpointProvider<Double>> {
    StateMachine<Double, E> getStateMachine();

    /**
     * Overrides the state setpoint with a dynamic supplier.
     */
    default void setSetpointOverride(DoubleSupplier override) {
        throw new UnsupportedOperationException("Setpoint overrides not supported by this mechanism");
    }

    /**
     * Clears any active setpoint override.
     */
    default void clearSetpointOverride() {
        setSetpointOverride(null);
    }

    /**
     * Suppresses output while the supplier evaluates to true.
     */
    default void setOutputSuppressor(BooleanSupplier suppressor) {
        throw new UnsupportedOperationException("Output suppression not supported by this mechanism");
    }

    /**
     * Clears any active output suppressor.
     */
    default void clearOutputSuppressor() {
        setOutputSuppressor(null);
    }
}
