package ca.frc6390.athena.mechanisms;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StateMachine<E extends Enum<E>> {
    
    private Trigger stateChanger;
    private E goalState, nextState;
    private BooleanSupplier atStateSupplier, changeStateSupplier;

    public StateMachine(E initialState, BooleanSupplier atStateSupplier){
        this.atStateSupplier = atStateSupplier;
        this.changeStateSupplier = () -> true;
        this.stateChanger = new Trigger(this::isAtState);
        this.stateChanger.onTrue(new InstantCommand(() -> 
        {
            goalState = nextState;
        }));
        setGoalState(initialState);
    }

    public void setGoalState(E state){
        setGoalState(state, () -> true);
    }

    public void setGoalState(E state, BooleanSupplier condition){
        nextState = state;
        changeStateSupplier = condition;
    }

    private boolean isAtState() {
        return this.changeStateSupplier.getAsBoolean();
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
