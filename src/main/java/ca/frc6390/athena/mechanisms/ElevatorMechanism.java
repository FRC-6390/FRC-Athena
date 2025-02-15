package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorMechanism extends Mechanism{
    private final ElevatorFeedforward feedforward;

    public ElevatorMechanism(MechanismConfig<? extends ElevatorMechanism> config, ElevatorFeedforward feedforward) {
        super(config);
        this.feedforward = feedforward;
    }

    @Override
    public double calculateFeedForward() {
        double value = feedforward.calculate(getVelocity());
        return isUseVoltage() ? value : value / 12d;
    }
    

    public static class StatefulElevatorMechanism<E extends Enum<E> & SetpointProvider> extends ElevatorMechanism {
    
        private final StateMachine<E> stateMachine;

        public StatefulElevatorMechanism(MechanismConfig<StatefulElevatorMechanism<E>> config,ElevatorFeedforward feedforward, E initialState) {
            super(config, feedforward);
            this.stateMachine = new StateMachine<>(initialState, this::atSetpoint);
        }

        @Override
        public double getSetpoint() {
            return stateMachine.getGoalState().getSetpoint();
        }

        @Override
        public void update() {
            stateMachine.update();  
            super.update();
        }

        public StateMachine<E> getStateMachine() {
            return stateMachine;
        }
    }
}
