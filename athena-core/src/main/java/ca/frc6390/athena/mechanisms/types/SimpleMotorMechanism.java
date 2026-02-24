package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.controllers.SimpleMotorFeedForwardsSendable;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SimpleMotorMechanism  extends Mechanism  {
    
    private final SimpleMotorFeedForwardsSendable feedforward;
    private final OutputType feedforwardOutputType;

    public SimpleMotorMechanism(MechanismConfig<? extends SimpleMotorMechanism> config) {
        super(config);
        MechanismConfig.FeedforwardProfile profile =
                config != null ? config.mechanismFeedforwardProfile(MechanismConfig.FeedforwardType.SIMPLE) : null;
        SimpleMotorFeedforward feedforward = profile != null ? profile.simple() : null;
        if (feedforward != null) {
            this.feedforward = new SimpleMotorFeedForwardsSendable(feedforward.getKs(), feedforward.getKv(), feedforward.getKa());
            this.feedforwardOutputType =
                    profile.outputType() != null ? profile.outputType() : OutputType.VOLTAGE;
            control().feedforwardEnabled(true);
        } else {
            this.feedforward = null;
            this.feedforwardOutputType = null;
        }
    }

    @Override
    public double calculateFeedForward() {
        if (feedforward == null) {
            return 0.0;
        }
        double valueVolts = feedforward.calculate(controllerSetpointVelocity());
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

    public static class StatefulSimpleMotorMechanism<E extends Enum<E> & SetpointProvider<Double>> extends SimpleMotorMechanism implements StatefulLike<E> {
    
        private final StatefulMechanismCore<StatefulSimpleMotorMechanism<E>, E> stateMachineCore;

        public StatefulSimpleMotorMechanism(MechanismConfig<StatefulSimpleMotorMechanism<E>> config,
                                            E initialState) {
            super(config);
            stateMachineCore = StatefulMechanismCore.fromConfig(initialState, this::atSetpoint, config);
        }

        @Override
        public StatefulLike.StateMachineSection<E> stateMachine() {
            return new StatefulLike.StateMachineSection<>(stateMachineCore);
        }

        @Override
        public void update() {
            stateMachineCore.updateMechanism(this);
            super.update();
        }

        @Override
        public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
            if (node == null) {
                return null;
            }
            stateMachineCore.getStateMachine().networkTables(node.child("StateMachine"));
            return super.networkTables(node);
        }

    }
}
