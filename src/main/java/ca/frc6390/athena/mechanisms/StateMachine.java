package ca.frc6390.athena.mechanisms;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

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

    private E goalState, nextState;
    private BooleanSupplier atStateSupplier, changeStateSupplier, nextchangeStateSupplier;
    private SendableChooser<E> chooser = new SendableChooser<>();

    public StateMachine(E initialState, BooleanSupplier atStateSupplier){
        chooser.setDefaultOption(initialState.name(), initialState);
        for (E state: goalState.getDeclaringClass().getEnumConstants()){
            chooser.addOption(state.name(), state);
        }
        this.goalState = initialState;
        this.nextState = initialState;
        this.atStateSupplier = atStateSupplier;
        this.changeStateSupplier = () -> true;
        this.nextchangeStateSupplier = () -> true;
        setGoalState(initialState);
    }

    public void setGoalState(E state){
        setGoalState(state, () -> true);
    }

    public void setGoalState(E state, BooleanSupplier condition){
        goalState = state;
        changeStateSupplier = condition;
    }

    public void setNextState(E state){
        setNextState(state, () -> true);
    }

    public void setNextState(E state, BooleanSupplier condition){
        nextState = state;
        nextchangeStateSupplier = condition;
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
        return nextState;
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
        if(shouldChangeState()){
            if (nextState != null) {
                goalState = nextState;
                changeStateSupplier = nextchangeStateSupplier;
            }
            nextchangeStateSupplier = null;
            nextState = null;
        }
    }

    public T getGoalStateSetpoint(){
        return getGoalState().getSetpoint();
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {

        layout.add("State Chooser", chooser);
        layout.add("Set State", new InstantCommand(() -> setGoalState(chooser.getSelected()))).withWidget(BuiltInWidgets.kCommand);

        layout.addString("Goal State", () -> this.getGoalState().name());
        layout.addString("Next State", () -> this.getNextState().name());
        layout.addBoolean("Should Change State", () -> this.shouldChangeState());

        ShuffleboardLayout statesLayout = layout.getLayout("States", BuiltInLayouts.kList);
        for (E state: goalState.getDeclaringClass().getEnumConstants()){
            statesLayout.add(state.name(), state.getSetpoint());
        }

        return layout;
    }
}
