package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.MotorController;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class StateMachineMechanism<E extends Enum<E> & SetpointProvider> extends ProfiledMechanism {
    
    public StateMachine<E> stateMachine;
    private boolean stateMachineEnabled;

    public StateMachineMechanism(MotorController motorController, ProfiledPIDController controller){
        super(motorController, controller);
    }

    public StateMachineMechanism(MotorController motorController, Encoder encoder, ProfiledPIDController controller){
        super(motorController, encoder, controller);
    }

    @SuppressWarnings("unchecked")
    public StateMachineMechanism<E> withUpperLimitSwitch(RunnableTrigger upperLimit){
        return (StateMachineMechanism<E>) super.withUpperLimitSwitch(upperLimit);
    }

    @SuppressWarnings("unchecked")
    public StateMachineMechanism<E> withUpperLimitSwitch(RunnableTrigger upperLimit, double maxPosition){
        return (StateMachineMechanism<E>) super.withUpperLimitSwitch(upperLimit, maxPosition);
    }

    @SuppressWarnings("unchecked")
    public StateMachineMechanism<E> withLowerLimitSwitch(RunnableTrigger lowerLimit){
        return (StateMachineMechanism<E>) super.withLowerLimitSwitch(lowerLimit);
    }

    @SuppressWarnings("unchecked")
    public StateMachineMechanism<E> withLowerLimitSwitch(RunnableTrigger lowerLimit, double minPosition){
        return (StateMachineMechanism<E>) super.withLowerLimitSwitch(lowerLimit, minPosition);
    }

    @SuppressWarnings("unchecked")
    public StateMachineMechanism<E> withHomeLimitSwitch(RunnableTrigger homeLimit){
        return (StateMachineMechanism<E>) super.withHomeLimitSwitch(homeLimit);
    }
    
    @SuppressWarnings("unchecked")
    public StateMachineMechanism<E> withHomeLimitSwitch(RunnableTrigger homeLimit, double homePosition){
        return (StateMachineMechanism<E>) super.withHomeLimitSwitch(homeLimit, homePosition);
    }

    public StateMachine<E> getStateMachine(){
        return stateMachine;
    }

    public void setStateMachineEnabled(boolean enabled){
        this.stateMachineEnabled = enabled;
    }

    public boolean isStateMachineEnabled(){
        return this.stateMachineEnabled;
    }

    @Override
    public void update() {
        if(stateMachineEnabled) {
            setSetpoint(stateMachine.getGoalState().getSetpoint());
        }
        super.update();
    }

}
