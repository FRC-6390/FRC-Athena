package ca.frc6390.athena.mechanisms;

import java.util.Objects;
import java.util.function.Consumer;

import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;

/**
 * Marker for mechanisms that own a state machine with a Double setpoint.
 * Implemented by all stateful mechanism variants (generic, arm, elevator, turret, etc.).
 *
 * @param <E> state enum type
 */
public interface StatefulLike<E extends Enum<E> & StateMachine.SetpointProvider<Double>> {
    StateMachineSection<E> stateMachine();

    default StatefulLike<E> stateMachine(Consumer<StateMachineSection<E>> section) {
        if (section != null) {
            section.accept(stateMachine());
        }
        return this;
    }

    final class StateMachineSection<E extends Enum<E> & StateMachine.SetpointProvider<Double>> {
        private final StatefulMechanismCore<? extends Mechanism, E> core;

        StateMachineSection(StatefulMechanismCore<? extends Mechanism, E> core) {
            this.core = Objects.requireNonNull(core, "core");
        }

        public StateMachineSection<E> queue(E state) {
            core.getStateMachine().queueState(state);
            return this;
        }

        public StateMachineSection<E> queue(double setpoint) {
            core.queueSetpoint(setpoint);
            return this;
        }

        public StateMachineSection<E> request(double setpoint) {
            core.requestSetpoint(setpoint);
            return this;
        }

        public StateMachineSection<E> request(E target) {
            core.requestState(target);
            return this;
        }

        public StateMachineSection<E> clear() {
            core.clear();
            return this;
        }

        public StateMachineSection<E> graph(StateGraph<E> stateGraph) {
            core.setStateGraph(stateGraph);
            return this;
        }

        public E goal() {
            return core.getStateMachine().getGoalState();
        }

        public E next() {
            return core.getStateMachine().getNextState();
        }

        public String queue() {
            return core.getStateMachine().getNextStateQueue();
        }

        public boolean queued(E state) {
            return core.getStateMachine().isQueued(state);
        }

        public boolean atGoal() {
            return core.getStateMachine().atGoalState();
        }

        @SafeVarargs
        public final boolean at(E... states) {
            return core.getStateMachine().atState(states);
        }

        public double setpoint() {
            return core.getSetpoint();
        }

        public StateGraph<E> graph() {
            return core.getStateMachine().getStateGraph();
        }

        public StateMachine<Double, E> machine() {
            return core.getStateMachine();
        }

        public StateMachineSnapshot<E> snapshot() {
            return new StateMachineSnapshot<>(
                    goal(),
                    next(),
                    queue(),
                    atGoal(),
                    setpoint(),
                    graph());
        }
    }

    record StateMachineSnapshot<E extends Enum<E> & StateMachine.SetpointProvider<Double>>(
            E goal,
            E next,
            String queue,
            boolean atGoal,
            double setpoint,
            StateGraph<E> graph) {}
}
