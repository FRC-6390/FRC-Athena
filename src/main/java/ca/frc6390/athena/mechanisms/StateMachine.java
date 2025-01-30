package ca.frc6390.athena.mechanisms;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StateMachine<E extends Enum<E>> {
    
    private Trigger stateChanger;
    private E goalState, nextState;
    private BooleanSupplier atStateSupplier;

    public StateMachine(E initialState, BooleanSupplier atStateSupplier){
        this.atStateSupplier = atStateSupplier;
        setGoalState(initialState);
    }

    public void setGoalState(E state){
        nextState = state;
        createStateChanger(() -> true);
    }

    public void setGoalState(E state, BooleanSupplier condition){
        nextState = state;
        createStateChanger(condition);
    }

    private void createStateChanger(BooleanSupplier condition) {
        this.stateChanger = new Trigger(condition);
        this.stateChanger.onTrue(new InstantCommand(() -> 
        {
            goalState = nextState;
        }));
    }

    public E getGoalState() {
        return goalState;
    }

    public boolean atGoalState() {
        return atStateSupplier.getAsBoolean();
    }

    public boolean atState(E state) {
        return atGoalState() && state == goalState;
    }
}
