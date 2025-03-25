package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SimpleMotorMechanism  extends Mechanism  {
    
    private final SimpleMotorFeedforward feedforward;

    public SimpleMotorMechanism(MechanismConfig<? extends SimpleMotorMechanism> config, SimpleMotorFeedforward feedforward) {
        super(config);
        this.feedforward = feedforward;
        setFeedforwardEnabled(true);
    }

    @Override
    public double calculateFeedForward() {
        double value = feedforward.calculate(getVelocity());
        return isUseVoltage() ? value : value / 12d;
    }

    @Override
    public SimpleMotorMechanism shuffleboard(String tab) {
        return (SimpleMotorMechanism) super.shuffleboard(tab);
    }
    public static class StatefulSimpleMotorMechanism<E extends Enum<E> & SetpointProvider<Double>> extends SimpleMotorMechanism {
    
        private final StatefulMechanismCore<StatefulSimpleMotorMechanism<E>, E> stateCore;

        public StatefulSimpleMotorMechanism(MechanismConfig<StatefulSimpleMotorMechanism<E>> config, SimpleMotorFeedforward feedforward, E initialState) {
            super(config, feedforward);
            stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.stateMachineDelay, config.stateActions);
        }

        @Override
        public double getSetpoint() {
            return stateCore.getSetpoint();
        }

        @Override
        public void update() {
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
    }
}
