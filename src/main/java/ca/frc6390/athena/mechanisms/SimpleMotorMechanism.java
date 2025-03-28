package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SimpleMotorMechanism  extends Mechanism  {
    
    private final SimpleMotorFeedforward feedforward;

    public SimpleMotorMechanism(MechanismConfig<? extends SimpleMotorMechanism> config, SimpleMotorFeedforward feedforward) {
        super(config);
        this.feedforward = new SimpleMotorFeedforward(feedforward.getKs(),feedforward.getKv(),feedforward.getKa());
        setFeedforwardEnabled(true);
    }

    @Override
    public double calculateFeedForward() {
        double value = feedforward.calculate(getControllerSetpointVelocity());
        return  isUseVoltage() ? value : value / 12d;
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        tab.add("FeedForwards", feedforward);
        return super.shuffleboard(tab, level);
    }

    @Override
    public SimpleMotorMechanism shuffleboard(String tab, SendableLevel level) {
        return (SimpleMotorMechanism) super.shuffleboard(tab, level);
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
        public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
            stateCore.shuffleboard(tab, level);
            return super.shuffleboard(tab, level);
        }

        @SuppressWarnings("unchecked")
        @Override
        public StatefulSimpleMotorMechanism<E> shuffleboard(String tab, SendableLevel level) {
            return (StatefulSimpleMotorMechanism<E>) super.shuffleboard(tab, level);
        }
    }
}
