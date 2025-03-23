package ca.frc6390.athena.mechanisms;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
public class StateMachine<T, E extends Enum<E> & SetpointProvider<T>>  implements RobotSendableDevice {
    
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
    private BooleanSupplier atStateSupplier, changeStateSupplier;
    private final SendableChooser<E> chooser = new SendableChooser<>();
    private final Queue<StateQueueEntry<E>> stateQueue = new LinkedList<>();

    public StateMachine(E initialState, BooleanSupplier atStateSupplier){
        chooser.setDefaultOption(initialState.name(), initialState);
        for (E state: initialState.getDeclaringClass().getEnumConstants()){
            chooser.addOption(state.name(), state);
        }
        this.goalState = initialState;
        this.atStateSupplier = atStateSupplier;
        this.changeStateSupplier = () -> true;
        queueState(initialState);
    }

    public void queueState(E state) {
        queueState(state, () -> true);
    }

    public void queueState(E state, BooleanSupplier condition) {
        stateQueue.add(new StateQueueEntry<>(state, condition));
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

    public Command atGoalStateCommand() {
        return Commands.waitUntil(this::atGoalState);
    }

    public boolean atGoalState() {
        return atStateSupplier.getAsBoolean();
    }

    public boolean atState(E state) {
        return atGoalState() && state == goalState;
    }

    public boolean atAnyState(@SuppressWarnings("unchecked") E... states) {
        return Arrays.asList(states).stream().anyMatch((state) -> atGoalState() && state == goalState);
    }

    public void update() {
        if (shouldChangeState() && !stateQueue.isEmpty()) {
            StateQueueEntry<E> entry = stateQueue.poll();
            goalState = entry.state;
            changeStateSupplier = entry.condition;
        }
    }

    public T getGoalStateSetpoint(){
        return getGoalState().getSetpoint();
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {

        layout.add("State Chooser", chooser);
        layout.add("Set State", new InstantCommand(() -> goalState = chooser.getSelected())).withWidget(BuiltInWidgets.kCommand);
        layout.add("Queue State", new InstantCommand(() -> queueState(chooser.getSelected()))).withWidget(BuiltInWidgets.kCommand);
        layout.add("Reset Queue", new InstantCommand(() -> resetQueue())).withWidget(BuiltInWidgets.kCommand);

        layout.addString("Goal State", () -> this.getGoalState().name());
        layout.addString("Next State", () -> this.getNextState().name());
        layout.addString("State Queue", () -> this.getNextStateQueue());
        layout.addBoolean("Should Change State", () -> this.shouldChangeState());

        ShuffleboardLayout statesLayout = layout.getLayout("States", BuiltInLayouts.kList);
        for (E state: goalState.getDeclaringClass().getEnumConstants()){
            statesLayout.add(state.name(), state.getSetpoint());
        }

        return layout;
    }
}
