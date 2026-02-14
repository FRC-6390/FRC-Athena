package ca.frc6390.athena.mechanisms;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;

/**
 * Marker for mechanisms that own a state machine with a Double setpoint.
 * Implemented by all stateful mechanism variants (generic, arm, elevator, turret, etc.).
 *
 * @param <E> state enum type
 */
public interface StatefulLike<E extends Enum<E> & StateMachine.SetpointProvider<Double>> {
    StatefulMechanismCore<? extends Mechanism, E> stateCore();

    default StateMachine<Double, E> getStateMachine() {
        return stateCore().getStateMachine();
    }

    default double getSetpoint() {
        return stateCore().getSetpoint();
    }

    /**
     * Overrides the state setpoint with a dynamic supplier.
     */
    default void setSetpointOverride(DoubleSupplier override) {
        stateCore().setSetpointOverride(override);
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
        stateCore().setOutputSuppressor(suppressor);
    }

    /**
     * Clears any active output suppressor.
     */
    default void clearOutputSuppressor() {
        setOutputSuppressor(null);
    }

    default void setStateGraph(StateGraph<E> stateGraph) {
        stateCore().setStateGraph(stateGraph);
    }

    default boolean updateStateCore(Mechanism mechanism) {
        return stateCore().updateMechanism(mechanism);
    }
}
