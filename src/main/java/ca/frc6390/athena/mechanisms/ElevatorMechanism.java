package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ElevatorMechanism extends Mechanism {

    private final ElevatorFeedforward feedforward;

    public ElevatorMechanism(MechanismConfig<? extends ElevatorMechanism> config, ElevatorFeedforward feedforward) {
        super(config);
        this.feedforward = feedforward;
        setFeedforwardEnabled(true);
    }

    @Override
    public double calculateFeedForward() {
        double value = feedforward.calculate(getVelocity());
        return  isUseVoltage() ? value : value / 12d;
    }

    @Override
    public ElevatorMechanism shuffleboard(String tab) {
        return (ElevatorMechanism) super.shuffleboard(tab);
    }
    
    public static class StatefulElevatorMechanism<E extends Enum<E> & StateMachine.SetpointProvider<Double>> extends ElevatorMechanism {

        private final StatefulMechanismCore<StatefulElevatorMechanism<E>, E> stateCore;

        public StatefulElevatorMechanism(MechanismConfig<StatefulElevatorMechanism<E>> config, ElevatorFeedforward feedforward, E initialState) {
            super(config, feedforward);
            stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.stateMachineDelay, config.stateActions);
        }

        @Override
        public double getSetpoint() {
            return stateCore.getSetpoint();
        }

        @Override
        public void periodic() {
            setSuppressMotorOutput(!stateCore.update(this));
            super.update();   
        }

        public StateMachine<Double, E> getStateMachine() {
            return stateCore.getStateMachine();
        }

        @Override
        public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
            stateCore.shuffleboard(tab);
            return super.shuffleboard(tab);
        }

        @SuppressWarnings("unchecked")
        @Override
        public StatefulElevatorMechanism<E> shuffleboard(String tab) {
            return (StatefulElevatorMechanism<E>) super.shuffleboard(tab);
        }
    }

}
