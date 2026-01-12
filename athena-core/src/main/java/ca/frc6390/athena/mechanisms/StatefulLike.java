package ca.frc6390.athena.mechanisms;

/**
 * Marker for mechanisms that own a state machine with a Double setpoint.
 * Implemented by all stateful mechanism variants (generic, arm, elevator, turret, etc.).
 *
 * @param <E> state enum type
 */
public interface StatefulLike<E extends Enum<E> & StateMachine.SetpointProvider<Double>> {
    StateMachine<Double, E> getStateMachine();
}
