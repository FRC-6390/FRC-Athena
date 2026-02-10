package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.controllers.SimpleMotorFeedForwardsSendable;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SimpleMotorMechanism  extends Mechanism  {
    
    private final SimpleMotorFeedForwardsSendable feedforward;
    private final OutputType feedforwardOutputType;

    public SimpleMotorMechanism(MechanismConfig<? extends SimpleMotorMechanism> config,
                                SimpleMotorFeedforward feedforward,
                                OutputType feedforwardOutputType) {
        super(config);
        if (feedforward != null) {
            this.feedforward = new SimpleMotorFeedForwardsSendable(feedforward.getKs(), feedforward.getKv(), feedforward.getKa());
            this.feedforwardOutputType = feedforwardOutputType != null ? feedforwardOutputType : OutputType.VOLTAGE;
            setFeedforwardEnabled(true);
        } else {
            this.feedforward = null;
            this.feedforwardOutputType = null;
            setFeedforwardEnabled(false);
        }
    }

    @Override
    public double calculateFeedForward() {
        if (feedforward == null) {
            return 0.0;
        }
        double valueVolts = feedforward.calculate(getControllerSetpointVelocity());
        return toOutput(feedforwardOutputType, valueVolts);
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return null;
        }
        if (feedforward == null) {
            return super.networkTables(node);
        }
        RobotNetworkTables.Node ff = node.child("Feedforward");
        ff.putDouble("ks", feedforward.getKs());
        ff.putDouble("kv", feedforward.getKv());
        ff.putDouble("ka", feedforward.getKa());
        return super.networkTables(node);
    }

    public SimpleMotorMechanism publishNetworkTables(String ownerHint) {
        super.publishNetworkTables(ownerHint);
        return this;
    }
    public static class StatefulSimpleMotorMechanism<E extends Enum<E> & SetpointProvider<Double>> extends SimpleMotorMechanism implements StatefulLike<E> {
    
        private final StatefulMechanismCore<StatefulSimpleMotorMechanism<E>, E> stateCore;

        public StatefulSimpleMotorMechanism(MechanismConfig<StatefulSimpleMotorMechanism<E>> config,
                                            SimpleMotorFeedforward feedforward,
                                            OutputType feedforwardOutputType,
                                            E initialState) {
            super(config, feedforward, feedforwardOutputType);
            stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.data().stateMachineDelay(),
                    config.stateActions,
                    config.enterStateHooks,
                    config.stateHooks,
                    config.exitStateHooks,
                    config.transitionHooks,
                    config.alwaysHooks,
                    config.exitAlwaysHooks,
                    config.inputs,
                    config.doubleInputs,
                    config.intInputs,
                    config.stringInputs,
                    config.pose2dInputs,
                    config.pose3dInputs,
                    config.objectInputs,
                    config.stateTriggerBindings);
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
        public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
            if (node == null) {
                return null;
            }
            getStateMachine().networkTables(node.child("StateMachine"));
            return super.networkTables(node);
        }

        @SuppressWarnings("unchecked")
        public StatefulSimpleMotorMechanism<E> publishNetworkTables(String ownerHint) {
            super.publishNetworkTables(ownerHint);
            return this;
        }
    }
}
