package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.arcp.ARCP;

import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmMechanism extends Mechanism {
    private final ArmFeedforward feedforward;
    private final OutputType feedforwardOutputType;

    public ArmMechanism(MechanismConfig<? extends ArmMechanism> config) {
        super(config);
        MechanismConfig.FeedforwardProfile profile =
                config != null ? config.mechanismFeedforwardProfile(MechanismConfig.FeedforwardType.ARM) : null;
        if (profile != null) {
            this.feedforward = profile.arm();
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
        double valueVolts =
                feedforward.calculate(
                        controllerSetpointPosition(),
                        controllerSetpointVelocity());
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
        ff.putDouble("kg", feedforward.getKg());
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
            publisher.put(ffRoot + "/kg", feedforward.getKg());
            publisher.put(ffRoot + "/kv", feedforward.getKv());
            publisher.put(ffRoot + "/ka", feedforward.getKa());
        }
        super.publishArcp(publisher, rootPath);
    }

    public static class StatefulArmMechanism<E extends Enum<E> & SetpointProvider<Double>> extends ArmMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulArmMechanism<E>, E> stateMachineCore;

        public StatefulArmMechanism(MechanismConfig<StatefulArmMechanism<E>> config,
                                    E initialState) {
            super(config);
            stateMachineCore = StatefulMechanismCore.fromConfig(initialState, this::atSetpoint, config);
            if (initialState != null && initialState.getSetpoint() != null) {
                control().setpoint(initialState.getSetpoint());
            }
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
