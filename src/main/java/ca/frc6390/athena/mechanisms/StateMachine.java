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
        queueState(state, () -> true);
    }

    public void queueState(E state, BooleanSupplier condition) {
        stateQueue.add(new StateQueueEntry<>(state, condition));
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

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoalState);
    }

    public Command waitUntil(@SuppressWarnings("unchecked") E... states) {
        return Commands.waitUntil(() -> this.atState(states));
    }

    public boolean atGoalState() {
        return atGoalDelayedOutput.getAsBoolean();
    }

    public boolean atState(@SuppressWarnings("unchecked") E... states) {
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

    public T getGoalStateSetpoint(){
        return getGoalState().getSetpoint();
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {

        if(level.equals(SendableLevel.DEBUG)){
            layout.add("State Chooser", chooser);
            layout.add("Set State", new InstantCommand(() -> goalState = chooser.getSelected())).withWidget(BuiltInWidgets.kCommand);
            layout.add("Queue State", new InstantCommand(() -> queueState(chooser.getSelected()))).withWidget(BuiltInWidgets.kCommand);
            layout.add("Reset Queue", new InstantCommand(() -> resetQueue())).withWidget(BuiltInWidgets.kCommand);

            ShuffleboardLayout statesLayout = layout.getLayout("States", BuiltInLayouts.kList);
            for (E state: goalState.getDeclaringClass().getEnumConstants()){
                statesLayout.add(state.name(), state.getSetpoint());
            }
        }
    
        layout.addString("Goal State", () -> this.getGoalState().name());
        layout.addString("Next State", () -> this.getNextState().name());
        layout.addString("State Queue", () -> this.getNextStateQueue());
        layout.addBoolean("Should Change State", () -> this.shouldChangeState());

        return layout;
    }
}
