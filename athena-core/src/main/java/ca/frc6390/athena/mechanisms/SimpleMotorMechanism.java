package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.controllers.SimpleMotorFeedForwardsSendable;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SimpleMotorMechanism  extends Mechanism  {
    
    private final SimpleMotorFeedForwardsSendable feedforward;

    public SimpleMotorMechanism(MechanismConfig<? extends SimpleMotorMechanism> config, SimpleMotorFeedforward feedforward) {
        super(config);
        this.feedforward = new SimpleMotorFeedForwardsSendable(feedforward.getKs(), feedforward.getKv(), feedforward.getKa());
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
    public static class StatefulSimpleMotorMechanism<E extends Enum<E> & SetpointProvider<Double>> extends SimpleMotorMechanism implements StatefulLike<E> {
    
        private final StatefulMechanismCore<StatefulSimpleMotorMechanism<E>, E> stateCore;

        public StatefulSimpleMotorMechanism(MechanismConfig<StatefulSimpleMotorMechanism<E>> config, SimpleMotorFeedforward feedforward, E initialState) {
            super(config, feedforward);
            stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.stateMachineDelay,
                    config.stateActions, config.stateHooks, config.alwaysHooks, config.inputs, config.doubleInputs, config.objectInputs);
        }

        @Override
        public double getSetpoint() {
            return stateCore.getSetpoint();
        }

        public void setSetpointOverride(DoubleSupplier override) {
            stateCore.setSetpointOverride(override);
        }

        public void clearSetpointOverride() {
            stateCore.clearSetpointOverride();
        }

        public void setOutputSuppressor(BooleanSupplier suppressor) {
            stateCore.setOutputSuppressor(suppressor);
        }

        public void clearOutputSuppressor() {
            stateCore.clearOutputSuppressor();
        }

        @Override
        public void update() {
            setSuppressMotorOutput(stateCore.update(this));
            super.update();
        }

        public StateMachine<Double, E> getStateMachine() {
            return stateCore.getStateMachine();
        }

        public void setStateGraph(StateGraph<E> stateGraph) {
            stateCore.setStateGraph(stateGraph);
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
