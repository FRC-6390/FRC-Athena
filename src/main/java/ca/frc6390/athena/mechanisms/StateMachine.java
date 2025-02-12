package ca.frc6390.athena.mechanisms;

import java.util.function.BooleanSupplier;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
public class StateMachine<E extends Enum<E> & SetpointProvider> {
    
    public interface SetpointProvider {
        double getSetpoint();
    }

    private E goalState, nextState;
    private BooleanSupplier atStateSupplier, changeStateSupplier;

    public StateMachine(E initialState, BooleanSupplier atStateSupplier){
        this.goalState = initialState;
        this.nextState = initialState;
        this.atStateSupplier = atStateSupplier;
        this.changeStateSupplier = () -> true;
        setGoalState(initialState);
    }

    public void setGoalState(E state){
        setGoalState(state, () -> true);
    }

    public void setGoalState(E state, BooleanSupplier condition){
        nextState = state;
        changeStateSupplier = condition;
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

    public boolean atGoalState() {
        return atStateSupplier.getAsBoolean();
    }

    public boolean atState(E state) {
        return atGoalState() && state == goalState;
    }

    public void update() {
        if(shouldChangeState()){
            goalState = nextState;
        }
    }
}
