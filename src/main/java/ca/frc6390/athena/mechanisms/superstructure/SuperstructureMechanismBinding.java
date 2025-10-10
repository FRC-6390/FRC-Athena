package ca.frc6390.athena.mechanisms.superstructure;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.util.SupplierUtil;

/**
 * Lightweight wrapper around a mechanism's {@link StateMachine}. Provides convenience methods for
 * queueing states and querying progress without exposing the underlying queueing logic to the rest of
 * the superstructure runtime.
 */
public final class SuperstructureMechanismBinding<E extends Enum<E> & SetpointProvider<?>> {

    private final StateMachine<?, E> stateMachine;

    SuperstructureMechanismBinding(StateMachine<?, E> stateMachine) {
        this.stateMachine = Objects.requireNonNull(stateMachine);
    }

    public StateMachine<?, E> stateMachine() {
        return stateMachine;
    }

    public void queue(E state) {
        stateMachine.queueState(state);
    }

    public void queue(E state, BooleanSupplier condition) {
        stateMachine.queueState(state, SupplierUtil.wrapBoolean(condition, true));
    }

    @SuppressWarnings("unchecked")
    void queueRaw(Enum<?> state, BooleanSupplier condition) {
        queue((E) state, condition);
    }

    @SafeVarargs
    public final boolean atState(E... states) {
        return stateMachine.atState(states);
    }

    public boolean atGoal() {
        return stateMachine.atGoalState();
    }

    public E goalState() {
        return stateMachine.getGoalState();
    }
}
