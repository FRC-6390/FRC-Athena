package ca.frc6390.athena.mechanisms;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.function.BooleanSupplier;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.statespec.StateBuilder;
import ca.frc6390.athena.mechanisms.statespec.StateCtx;
import ca.frc6390.athena.mechanisms.statespec.StateSeed;
import ca.frc6390.athena.mechanisms.statespec.StateSeedProvider;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
public class StateMachine<T, E extends Enum<E> & SetpointProvider<T>>  implements RobotSendableDevice {
    
    private final DelayedOutput atGoalDelayedOutput;

    public interface SetpointProvider<T> {
        T getSetpoint();
    }

    private static class StateQueueEntry<E> {
        final E state;
        final BooleanSupplier condition;
        
        StateQueueEntry(E state, BooleanSupplier condition) {
            this.state = state;
            this.condition = condition;
        }
    }

    private E goalState;
    private BooleanSupplier changeStateSupplier;
    private final SendableChooser<E> chooser = new SendableChooser<>();
    private final Queue<StateQueueEntry<E>> stateQueue = new LinkedList<>();
    private StateGraph<E> stateGraph;
    private double goalStateEnteredSeconds;

    public StateMachine(E initialState, BooleanSupplier atStateSupplier){
        chooser.setDefaultOption(initialState.name(), initialState);
        for (E state: initialState.getDeclaringClass().getEnumConstants()){
            chooser.addOption(state.name(), state);
        }
        this.goalState = initialState;
        this.goalStateEnteredSeconds = Timer.getFPGATimestamp();
        this.changeStateSupplier = () -> true;
        this.atGoalDelayedOutput = new DelayedOutput(atStateSupplier, 0);
        queue(initialState);
    }

    public void setAtStateDelay(double delay){
        atGoalDelayedOutput.setDelay(delay);
    }

    public void queue(E state) {
        queue(state, StateGraph.Guards.always());
    }

    public void queue(E state, BooleanSupplier condition) {
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(condition, "condition");

        if (stateGraph == null) {
            enqueue(state, condition);
            return;
        }

        E start = tailState();
        List<E> path = expandPath(start, state);
        if (path.isEmpty() && !start.equals(state)) {
            path = List.of(state);
        }

        if (path.isEmpty()) {
            enqueue(state, condition);
            return;
        }

        for (int i = 0; i < path.size(); i++) {
            E next = path.get(i);
            BooleanSupplier guard = edgeGuard(start, next);
            if (i == path.size() - 1) {
                guard = StateGraph.Guards.allOf(guard, condition);
            }
            enqueue(next, guard);
            start = next;
        }
    }

    /**
     * Returns true if the state is currently present in the queue.
     */
    public boolean isQueued(E state) {
        if (state == null || stateQueue.isEmpty()) {
            return false;
        }
        for (StateQueueEntry<E> entry : stateQueue) {
            if (entry != null && entry.state == state) {
                return true;
            }
        }
        return false;
    }

    public void setStateGraph(StateGraph<E> stateGraph) {
        this.stateGraph = stateGraph;
    }

    public StateGraph<E> getStateGraph() {
        return stateGraph;
    }

    public void force(E target) {
        force(target, false);
    }

    public void force(E target, boolean append) {
        Objects.requireNonNull(target, "target");
        if (!append) {
            resetQueue();
        }
        E start = append ? tailState() : goalState;
        for (E next : expandPath(start, target)) {
            queue(next, edgeGuard(start, next));
            start = next;
        }
    }

    public BooleanSupplier getChangeStateSupplier(){
        return changeStateSupplier;
    }

    private boolean shouldChangeState() {
        return changeStateSupplier.getAsBoolean();
    }

    public E getGoalState() {
        return goalState;
    }

    public SendableChooser<E> chooser() {
        return chooser;
    }

    public boolean isGoalState(E state) {
        return goalState.equals(state);
    }

    public E getNextState() {
        if (!stateQueue.isEmpty()) {
            return stateQueue.peek().state;
        }
        return goalState;
    }

    public String getNextStateQueue() {
        if (stateQueue.isEmpty()) {
            return "None";
        }
        StringBuilder queueString = new StringBuilder();
        for (StateQueueEntry<E> entry : stateQueue) {
            if (entry == null || entry.state == null) {
                continue;
            }
            if (queueString.length() > 0) {
                queueString.append(", ");
            }
            queueString.append(entry.state.name());
        }
        return queueString.length() > 0 ? queueString.toString() : "None";
    }

    public void resetQueue(){
        stateQueue.clear();
    }

    private E tailState() {
        @SuppressWarnings("unchecked")
        LinkedList<StateQueueEntry<E>> list = (LinkedList<StateQueueEntry<E>>) stateQueue;
        if (list.isEmpty()) {
            return goalState;
        }
        return list.peekLast().state;
    }

    private List<E> expandPath(E start, E target) {
        if (stateGraph == null) {
            if (start.equals(target)) {
                return List.of();
            }
            return List.of(target);
        }
        List<E> expanded = stateGraph.expand(start, target);
        if (expanded.isEmpty() && !start.equals(target)) {
            return List.of(target);
        }
        return expanded;
    }

    private BooleanSupplier edgeGuard(E from, E to) {
        if (stateGraph == null) {
            return StateGraph.Guards.always();
        }
        return stateGraph.guardFor(from, to);
    }

    private void enqueue(E state, BooleanSupplier condition) {
        stateQueue.add(new StateQueueEntry<>(state, condition));
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }

    @SafeVarargs
    public final Command waitUntil(E... states) {
        return Commands.waitUntil(() -> this.at(states));
    }

    public boolean atGoal() {
        return atGoalDelayedOutput.getAsBoolean();
    }

    @SafeVarargs
    public final boolean at(E... states) {
        return Arrays.stream(states).anyMatch((state) -> atGoal() && isGoalState(state));
    }

    public void update() {
        if (!stateQueue.isEmpty()) {
            changeStateSupplier = stateQueue.peek().condition;
            if (shouldChangeState()) {
                StateQueueEntry<E> entry = stateQueue.poll();
                goalState = entry.state;
                goalStateEnteredSeconds = Timer.getFPGATimestamp();
            }
        }
        applyDslTransitionIfReady();
    }

    public T getGoalStateSetpoint(){
        return getGoalState().getSetpoint();
    }

    public double goalStateTimeSeconds() {
        return Math.max(0.0, Timer.getFPGATimestamp() - goalStateEnteredSeconds);
    }

    private void applyDslTransitionIfReady() {
        StateSeed<E> seed = seedFor(goalState);
        if (seed == null || seed.kind() != StateSeed.Kind.DSL || seed.dsl() == null) {
            return;
        }

        StateBuilder<E> builder = new StateBuilder<>();
        StateBuilder<E> applied = seed.dsl().apply(builder);
        StateBuilder<E> resolved = applied != null ? applied : builder;
        if (resolved.until() == null || resolved.next() == null) {
            return;
        }

        StateCtx<E> ctx = new StateCtx<>() {
            @Override
            public E state() {
                return goalState;
            }

            @Override
            public double timeInState() {
                return goalStateTimeSeconds();
            }
        };

        boolean shouldQueue;
        try {
            shouldQueue = resolved.until().test(ctx);
        } catch (RuntimeException ex) {
            return;
        }
        if (shouldQueue && !goalState.equals(resolved.next()) && !isQueued(resolved.next())) {
            queue(resolved.next());
        }
    }

    @SuppressWarnings("unchecked")
    private StateSeed<E> seedFor(E state) {
        if (!(state instanceof StateSeedProvider<?> provider)) {
            return null;
        }
        return ((StateSeedProvider<E>) provider).seed();
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        if (!node.robot().isPublishingEnabled()) {
            return node;
        }
        E goal = getGoalState();
        E next = getNextState();
        node.putString("goalState", goal != null ? goal.name() : "");
        node.putString("nextState", next != null ? next.name() : "");
        node.putString("queue", getNextStateQueue());
        node.putBoolean("shouldChangeState", shouldChangeState());
        node.putBoolean("atGoalState", atGoal());
        return node;
    }
}
