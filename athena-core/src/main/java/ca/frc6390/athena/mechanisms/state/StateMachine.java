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
import ca.frc6390.athena.mechanisms.statespec.StateId;
import ca.frc6390.athena.mechanisms.statespec.StateNames;
import ca.frc6390.athena.mechanisms.statespec.StateSeed;
import ca.frc6390.athena.mechanisms.statespec.StateSpecAccess;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.arcp.ARCP;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
public class StateMachine<T, E>  implements RobotSendableDevice {
    
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
    private final List<E> availableStates;
    private BooleanSupplier changeStateSupplier;
    private final SendableChooser<E> chooser = new SendableChooser<>();
    private final Queue<StateQueueEntry<E>> stateQueue = new LinkedList<>();
    private StateGraph<E> stateGraph;
    private double goalStateEnteredSeconds;
    private boolean arcpAppendMode;

    public StateMachine(E initialState, BooleanSupplier atStateSupplier){
        this.availableStates = availableStates(initialState);
        chooser.setDefaultOption(stateNameOrBlank(initialState), initialState);
        for (E state: availableStates){
            if (!Objects.equals(state, initialState)) {
                chooser.addOption(stateNameOrBlank(state), state);
            }
        }
        this.goalState = initialState;
        this.goalStateEnteredSeconds = Timer.getFPGATimestamp();
        this.changeStateSupplier = () -> true;
        this.atGoalDelayedOutput = new DelayedOutput(atStateSupplier, 0);
        this.arcpAppendMode = false;
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
            queueString.append(stateNameOrBlank(entry.state));
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
        return StateSpecAccess.setpoint(getGoalState());
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
        E nextState = resolved.next() != null ? resolved.next() : StateSpecAccess.resolve(goalState, resolved.nextName());
        if (resolved.until() == null || nextState == null) {
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
        if (shouldQueue && !goalState.equals(nextState) && !isQueued(nextState)) {
            queue(nextState);
        }
    }

    private StateSeed<E> seedFor(E state) {
        return StateSpecAccess.seed(state);
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
        node.putString("goalState", stateNameOrBlank(goal));
        node.putString("nextState", stateNameOrBlank(next));
        node.putString("queue", getNextStateQueue());
        node.putBoolean("shouldChangeState", shouldChangeState());
        node.putBoolean("atGoalState", atGoal());
        return node;
    }

    public void publishArcp(ARCP publisher, String rootPath) {
        if (publisher == null || rootPath == null || rootPath.isBlank()) {
            return;
        }
        publisher.writableString(rootPath + "/goalState").onSet(this::applyArcpGoalState);
        publisher.writableBoolean(rootPath + "/appendMode").onSetBoolean(append -> arcpAppendMode = append);
        publisher.command(rootPath + "/command/clearQueue").onInvoke(this::resetQueue);

        E goal = getGoalState();
        E next = getNextState();
        publisher.put(rootPath + "/currentState", stateNameOrBlank(goal));
        publisher.put(rootPath + "/goalState", stateNameOrBlank(goal));
        publisher.put(rootPath + "/nextState", stateNameOrBlank(next));
        publisher.put(rootPath + "/queue", getNextStateQueue());
        publisher.put(rootPath + "/shouldChangeState", shouldChangeState());
        publisher.put(rootPath + "/atGoalState", atGoal());
        publisher.put(rootPath + "/appendMode", arcpAppendMode);

        String[] names = new String[availableStates.size()];
        for (int i = 0; i < availableStates.size(); i++) {
            names[i] = stateNameOrBlank(availableStates.get(i));
        }
        publisher.put(rootPath + "/availableStates", names);
    }

    private void applyArcpGoalState(String rawState) {
        if (rawState == null || rawState.isBlank() || goalState == null) {
            return;
        }
        String requested = rawState.trim();
        for (E value : availableStates) {
            if (!stateNameOrBlank(value).equalsIgnoreCase(requested)) {
                continue;
            }
            force(value, arcpAppendMode);
            return;
        }
    }

    private static String stateNameOrBlank(Object state) {
        String name = StateNames.name(state);
        return name != null ? name : "";
    }

    @SuppressWarnings("unchecked")
    private static <E> List<E> availableStates(E initialState) {
        if (initialState == null) {
            return List.of();
        }
        if (initialState instanceof StateId stateId) {
            return (List<E>) List.copyOf(stateId.owner().states());
        }
        if (initialState instanceof Enum<?> enumState) {
            Object[] constants = enumState.getDeclaringClass().getEnumConstants();
            if (constants != null) {
                return Arrays.stream(constants)
                        .map(value -> (E) value)
                        .toList();
            }
        }
        return List.of(initialState);
    }
}
