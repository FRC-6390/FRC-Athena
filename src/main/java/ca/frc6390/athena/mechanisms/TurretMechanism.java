package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

    @Override
    public TurretMechanism shuffleboard(String tab) {
        return (TurretMechanism) super.shuffleboard(tab);
    }
    public static class StatefulTurretMechanism<E extends Enum<E> & SetpointProvider<Double>> extends TurretMechanism {
    
        private final StateMachine<Double, E> stateMachine;

        public StatefulTurretMechanism(MechanismConfig<StatefulTurretMechanism<E>> config,SimpleMotorFeedforward feedforward, E initialState) {
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
        public StatefulTurretMechanism<E> shuffleboard(String tab) {
            return (StatefulTurretMechanism<E>) super.shuffleboard(tab);
        }

        @Override
        public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
            stateMachine.shuffleboard(tab.getLayout("State Machine", BuiltInLayouts.kList));
            return super.shuffleboard(tab);
        }
    }
}
