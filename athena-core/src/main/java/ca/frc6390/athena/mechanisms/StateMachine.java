package ca.frc6390.athena.mechanisms;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
public class StateMachine<T, E extends Enum<E> & SetpointProvider<T>>  implements RobotSendableDevice {
    
    private final DelayedOutput atGoalDelayedOutput;
    private double shuffleboardPeriodSeconds = ca.frc6390.athena.core.RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();

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

    public StateMachine(E initialState, BooleanSupplier atStateSupplier){
        chooser.setDefaultOption(initialState.name(), initialState);
        for (E state: initialState.getDeclaringClass().getEnumConstants()){
            chooser.addOption(state.name(), state);
        }
        this.goalState = initialState;
        this.changeStateSupplier = () -> true;
        this.atGoalDelayedOutput = new DelayedOutput(atStateSupplier, 0);
        queueState(initialState);
    }

    public void setAtStateDelay(double delay){
        atGoalDelayedOutput.setDelay(delay);
    }

    public void queueState(E state) {
        queueState(state, StateGraph.Guards.always());
    }

    public void queueState(E state, BooleanSupplier condition) {
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

    public void setStateGraph(StateGraph<E> stateGraph) {
        this.stateGraph = stateGraph;
    }

    public StateGraph<E> getStateGraph() {
        return stateGraph;
    }

    public void requestState(E target) {
        requestState(target, false);
    }

    public void requestState(E target, boolean append) {
        Objects.requireNonNull(target, "target");
        if (!append) {
            resetQueue();
        }
        E start = append ? tailState() : goalState;
        for (E next : expandPath(start, target)) {
            queueState(next, edgeGuard(start, next));
            start = next;
        }
    }

    @SafeVarargs
    public final void requestStates(boolean append, E... targets) {
        Objects.requireNonNull(targets, "targets");
        if (!append) {
            resetQueue();
        }
        E start = append ? tailState() : goalState;
        for (E target : targets) {
            Objects.requireNonNull(target, "targets cannot contain null entries.");
            if (start.equals(target)) {
                continue;
            }
            for (E next : expandPath(start, target)) {
                queueState(next, edgeGuard(start, next));
                start = next;
            }
            start = target;
        }
    }

    @SafeVarargs
    public final void requestStates(E... targets) {
        requestStates(false, targets);
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
        return stateQueue.stream()
                .map(entry -> entry.state.name())
                .collect(Collectors.joining(", "));
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
        return Commands.waitUntil(this::atGoalState);
    }

    @SafeVarargs
    public final Command waitUntil(E... states) {
        return Commands.waitUntil(() -> this.atState(states));
    }

    public boolean atGoalState() {
        return atGoalDelayedOutput.getAsBoolean();
    }

    @SafeVarargs
    public final boolean atState(E... states) {
        return Arrays.stream(states).anyMatch((state) -> atGoalState() && isGoalState(state));
    }

    public void update() {
        if (!stateQueue.isEmpty()) {
            changeStateSupplier = stateQueue.peek().condition;
            if (shouldChangeState()) {
                StateQueueEntry<E> entry = stateQueue.poll();
                goalState = entry.state;
            }
        }
    }

    @Override
    public double getShuffleboardPeriodSeconds() {
        return shuffleboardPeriodSeconds;
    }

    @Override
    public void setShuffleboardPeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds)) {
            return;
        }
        shuffleboardPeriodSeconds = periodSeconds;
    }

    public T getGoalStateSetpoint(){
        return getGoalState().getSetpoint();
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;

        if(level.equals(SendableLevel.DEBUG)){
            layout.add("State Chooser", chooser);
            layout.add("Set State", new InstantCommand(() -> goalState = chooser.getSelected())).withWidget(BuiltInWidgets.kCommand);
            layout.add("Queue State", new InstantCommand(() -> queueState(chooser.getSelected()))).withWidget(BuiltInWidgets.kCommand);
            layout.add("Reset Queue", new InstantCommand(() -> resetQueue())).withWidget(BuiltInWidgets.kCommand);

            ShuffleboardLayout statesLayout = layout.getLayout("States", BuiltInLayouts.kList);
            for (E state: goalState.getDeclaringClass().getEnumConstants()){
                Object setpoint = state.getSetpoint();
                if (setpoint instanceof Number number) {
                    statesLayout.addNumber(state.name(),
                            ca.frc6390.athena.core.RobotSendableSystem.rateLimit(number::doubleValue, period));
                } else if (setpoint instanceof Boolean bool) {
                    statesLayout.addBoolean(state.name(),
                            ca.frc6390.athena.core.RobotSendableSystem.rateLimit(() -> bool, period));
                } else {
                    statesLayout.addString(state.name(),
                            ca.frc6390.athena.core.RobotSendableSystem.rateLimit(() -> String.valueOf(setpoint), period));
                }
            }
        }
    
        layout.addString("Goal State",
                ca.frc6390.athena.core.RobotSendableSystem.rateLimit(() -> this.getGoalState().name(), period));
        layout.addString("Next State",
                ca.frc6390.athena.core.RobotSendableSystem.rateLimit(() -> this.getNextState().name(), period));
        layout.addString("State Queue",
                ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getNextStateQueue, period));
        layout.addBoolean("Should Change State",
                ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::shouldChangeState, period));

        return layout;
    }
}
