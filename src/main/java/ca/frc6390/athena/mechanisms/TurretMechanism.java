package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TurretMechanism  extends Mechanism  {
    private final SimpleMotorFeedforward feedforward;

    public TurretMechanism(MechanismConfig<? extends TurretMechanism> config, SimpleMotorFeedforward feedforward) {
        super(config);
        this.feedforward = feedforward;
    }

    @Override
    public double calculateFeedForward() {
        double value = feedforward.calculate(getVelocity());
        return isUseVoltage() ? value : value / 12d;
    }

    public static class StatefulTurretMechanism<E extends Enum<E> & SetpointProvider> extends TurretMechanism {
    
        private final StateMachine<E> stateMachine;

        public StatefulTurretMechanism(MechanismConfig<StatefulTurretMechanism<E>> config,SimpleMotorFeedforward feedforward, E initialState) {
            super(config, feedforward);
            this.stateMachine = new StateMachine<>(initialState, () -> true);
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
