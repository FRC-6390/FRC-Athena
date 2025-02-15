package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.MotorController;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ElevatorMechanism<E extends Enum<E> & SetpointProvider> extends StateMachineMechanism<E>{

    private final ElevatorFeedforward feedforward;
    private boolean feedforwardEnabled = false;

    public record ElevatorMechanismConfig() {
    }

    public ElevatorMechanism(MotorController motorController, Encoder encoder, ProfiledPIDController controller, E initialState) {
        super(motorController, encoder, controller, initialState);
        this.feedforward = new ElevatorFeedforward(0,0,0,0,0);
    }

    public ElevatorMechanism<E> withUpperLimitSwitch(RunnableTrigger upperLimit){
        return (ElevatorMechanism<E>) super.withUpperLimitSwitch(upperLimit);
    }

    public ElevatorMechanism<E> withUpperLimitSwitch(RunnableTrigger upperLimit, double maxPosition){
        return (ElevatorMechanism<E>) super.withUpperLimitSwitch(upperLimit, maxPosition);
    }

    public ElevatorMechanism<E> withLowerLimitSwitch(RunnableTrigger lowerLimit){
        return (ElevatorMechanism<E>) super.withLowerLimitSwitch(lowerLimit);
    }

    public ElevatorMechanism<E> withLowerLimitSwitch(RunnableTrigger lowerLimit, double minPosition){
        return (ElevatorMechanism<E>) super.withLowerLimitSwitch(lowerLimit, minPosition);
    }

    public ElevatorMechanism<E> withHomeLimitSwitch(RunnableTrigger homeLimit){
        return (ElevatorMechanism<E>) super.withHomeLimitSwitch(homeLimit);
    }
    
    public ElevatorMechanism<E> withHomeLimitSwitch(RunnableTrigger homeLimit, double homePosition){
        return (ElevatorMechanism<E>) super.withHomeLimitSwitch(homeLimit, homePosition);
    }

    public void setFeedforwardEnabled(boolean feedforwardEnabled) {
        this.feedforwardEnabled = feedforwardEnabled;
    }

    public boolean isFeedforwardEnabled() {
        return feedforwardEnabled;
    }
    
    @Override
    public double calculateSpeed(){
        return super.calculateSpeed() + feedforward.calculate(getVelocity());
    }

    public void update(){
        super.update();
    }
}
