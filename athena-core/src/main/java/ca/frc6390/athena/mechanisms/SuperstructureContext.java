package ca.frc6390.athena.mechanisms;

import java.util.function.Function;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Read-only view of the composite that guards can use to inspect child mechanisms.
 *
 * @param <SP> setpoint tuple type produced by the superstate enum
 */
public interface SuperstructureContext<SP> {

    SP setpoint();

    <E extends Enum<E> & SetpointProvider<Double>> StatefulMechanism<E> mechanism(Function<SP, E> mapper);

    <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureMechanism<CS, CSP> superstructure(Function<SP, CS> mapper);

    /**
     * Returns a typed accessor for child mechanisms and nested superstructures.
     */
    SuperstructureMechanismsView<SP> getMechanisms();

    /**
     * Returns the value of a named external input (added via the superstructure config).
     */
    boolean input(String key);
}
