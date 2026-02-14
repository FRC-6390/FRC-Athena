package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.controllers.ArmFeedforwardSendable;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;

import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmMechanism extends Mechanism {
    private final ArmFeedforwardSendable feedforward;
    private final OutputType feedforwardOutputType;

    public ArmMechanism(MechanismConfig<? extends ArmMechanism> config,
                        ArmFeedforward feedforward,
                        OutputType feedforwardOutputType) {
        super(config);
        if (feedforward != null) {
            this.feedforward = new ArmFeedforwardSendable(feedforward.getKs(),feedforward.getKg(),feedforward.getKv(),feedforward.getKa());
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
        double valueVolts = feedforward.calculate(getControllerSetpointPosition(), getControllerSetpointVelocity());
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

    public ArmMechanism publishNetworkTables(String ownerHint) {
        super.publishNetworkTables(ownerHint);
        return this;
    }
    
    public static class StatefulArmMechanism<E extends Enum<E> & SetpointProvider<Double>> extends ArmMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulArmMechanism<E>, E> stateCore;

        public StatefulArmMechanism(MechanismConfig<StatefulArmMechanism<E>> config,
                                    ArmFeedforward feedforward,
                                    OutputType feedforwardOutputType,
                                    E initialState) {
            super(config, feedforward, feedforwardOutputType);
            stateCore = StatefulMechanismCore.fromConfig(initialState, this::atSetpoint, config);
        }

        @Override
        public StatefulMechanismCore<StatefulArmMechanism<E>, E> stateCore() {
            return stateCore;
        }

        @Override
        public void update() {
            setSuppressMotorOutput(updateStateCore(this));
            super.update();
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
        public StatefulArmMechanism<E> publishNetworkTables(String ownerHint) {
            super.publishNetworkTables(ownerHint);
            return this;
        }
    }
}
