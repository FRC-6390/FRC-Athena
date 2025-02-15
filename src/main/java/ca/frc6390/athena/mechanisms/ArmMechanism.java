package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmMechanism extends Mechanism {
    private final ArmFeedforward feedforward;

    public ArmMechanism(MechanismConfig<? extends ArmMechanism> config, ArmFeedforward feedforward) {
        super(config);
        this.feedforward = feedforward;
    }

    @Override
    public double calculateFeedForward() {
        double value = feedforward.calculate(getRotation2d().getRadians(), getVelocity());
        return isUseVoltage() ? value : value / 12d;
    }


    public static class StatefulArmMechanism<E extends Enum<E> & SetpointProvider> extends ArmMechanism {

        private final StateMachine<E> stateMachine;

        public StatefulArmMechanism(MechanismConfig<StatefulArmMechanism<E>> config,ArmFeedforward feedforward, E initialState) {
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



