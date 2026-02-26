package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.controllers.SimpleMotorFeedForwardsSendable;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.arcp.ARCP;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SimpleMotorMechanism  extends Mechanism  {
    
    private final SimpleMotorFeedForwardsSendable feedforward;

    public SimpleMotorMechanism(MechanismConfig<? extends SimpleMotorMechanism> config) {
        super(config);
        MechanismConfig.FeedforwardProfile profile =
                config != null ? config.mechanismFeedforwardProfile(MechanismConfig.FeedforwardType.SIMPLE) : null;
        SimpleMotorFeedforward feedforward = profile != null ? profile.simple() : null;
        if (feedforward != null) {
            this.feedforward = new SimpleMotorFeedForwardsSendable(feedforward.getKs(), feedforward.getKv(), feedforward.getKa());
            control().feedforwardEnabled(true);
        } else {
            this.feedforward = null;
        }
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

    @Override
    public void publishArcp(ARCP publisher, String rootPath) {
        if (publisher == null || rootPath == null || rootPath.isBlank()) {
            return;
        }
        if (feedforward != null) {
            String ffRoot = rootPath + "/Feedforward";
            publisher.put(ffRoot + "/ks", feedforward.getKs());
            publisher.put(ffRoot + "/kv", feedforward.getKv());
            publisher.put(ffRoot + "/ka", feedforward.getKa());
        }
        super.publishArcp(publisher, rootPath);
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

        @Override
        public void publishArcp(ARCP publisher, String rootPath) {
            if (publisher == null || rootPath == null || rootPath.isBlank()) {
                return;
            }
            stateMachineCore.getStateMachine().publishArcp(publisher, rootPath + "/Control/StateMachine");
            stateMachineCore.getStateMachine().publishArcp(publisher, rootPath + "/StateMachine");
            super.publishArcp(publisher, rootPath);
        }

    }
}
