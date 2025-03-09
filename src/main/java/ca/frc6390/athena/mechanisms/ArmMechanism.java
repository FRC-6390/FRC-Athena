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

    @Override
    public ArmMechanism shuffleboard(String tab) {
        return (ArmMechanism) super.shuffleboard(tab);
    }
    
    public static class StatefulArmMechanism<E extends Enum<E> & SetpointProvider<Double>> extends ArmMechanism {

        private final StateMachine<Double, E> stateMachine;

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

        public StateMachine<Double, E> getStateMachine() {
            return stateMachine;
        }

        @SuppressWarnings("unchecked")
        @Override
        public StatefulArmMechanism<E> shuffleboard(String tab) {
            return (StatefulArmMechanism<E>) super.shuffleboard(tab);
        }
    }
}



