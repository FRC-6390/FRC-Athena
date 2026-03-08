package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.arcp.ARCP;
import ca.frc6390.athena.mechanisms.statespec.StateSpecAccess;

/**
 * Simple motor mechanism intended for flywheel shooters and rollers.
 */
public class FlywheelMechanism extends SimpleMotorMechanism {

    public FlywheelMechanism(MechanismConfig<? extends FlywheelMechanism> config) {
        super(config);
    }

    public static class StatefulFlywheelMechanism<E extends Enum<E>>
            extends FlywheelMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulFlywheelMechanism<E>, E> stateMachineCore;

        public StatefulFlywheelMechanism(MechanismConfig<StatefulFlywheelMechanism<E>> config,
                                         E initialState) {
            super(config);
            stateMachineCore = StatefulMechanismCore.fromConfig(initialState, this::atSetpoint, config);
            Double initialSetpoint = StateSpecAccess.setpoint(initialState);
            if (initialSetpoint != null) {
                control().setpoint(initialSetpoint);
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
