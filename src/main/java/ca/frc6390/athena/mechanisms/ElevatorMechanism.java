package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ElevatorMechanism extends Mechanism {

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

    @Override
    public ElevatorMechanism shuffleboard(String tab) {
        return (ElevatorMechanism) super.shuffleboard(tab);
    }
    
    public static class StatefulElevatorMechanism<E extends Enum<E> & SetpointProvider<Double>> extends ElevatorMechanism {
    
        private final StateMachine<Double, E> stateMachine;

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

        public StateMachine<Double, E> getStateMachine() {
            return stateMachine;
        }

        @SuppressWarnings("unchecked")
        @Override
        public StatefulElevatorMechanism<E> shuffleboard(String tab) {
            return (StatefulElevatorMechanism<E>) super.shuffleboard(tab);
        }


        @Override
        public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
            stateMachine.shuffleboard(tab.getLayout("State Machine", BuiltInLayouts.kList));
            return super.shuffleboard(tab);
        }
    }
}
