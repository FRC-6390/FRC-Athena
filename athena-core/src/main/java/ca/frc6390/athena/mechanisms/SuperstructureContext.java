package ca.frc6390.athena.mechanisms;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Read-only view of the composite that constraints can use to inspect child mechanisms.
 *
 * @param <SP> setpoint tuple type produced by the superstate enum
 */
public interface SuperstructureContext<SP> {

    SP setpoint();

    <E extends Enum<E> & SetpointProvider<Double>> StatefulLike<E> mechanism(Function<SP, E> mapper);

    <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureMechanism<CS, CSP> superstructure(Function<SP, CS> mapper);

    /**
     * Returns a typed accessor for child mechanisms and nested superstructures.
     */
    SuperstructureMechanismsView<SP> getMechanisms();

    /**
     * Returns the value of a named external input (added via the superstructure config).
     */
    boolean input(String key);

    /**
     * Returns the value of a named double input (added via the superstructure config).
     */
    double doubleInput(String key);

    /**
     * Returns the supplier for a named double input.
     */
    DoubleSupplier doubleInputSupplier(String key);

    /**
     * Returns a named object input using the requested type.
     */
    <T> T objectInput(String key, Class<T> type);

    /**
     * Returns the supplier for a named object input.
     */
    <T> Supplier<T> objectInputSupplier(String key, Class<T> type);

    /**
     * Returns the base setpoint mapped from the current superstate setpoint.
     */
    <E extends Enum<E> & StateMachine.SetpointProvider<Double>> double mappedSetpoint(Function<SP, E> mapper);
}
