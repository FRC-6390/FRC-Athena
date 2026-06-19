package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.arcp.ARCP;
import ca.frc6390.athena.mechanisms.statespec.StateSpecAccess;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Simple motor mechanism intended for flywheel shooters and rollers.
 */
public class FlywheelMechanism extends SimpleMotorMechanism {

    protected FlywheelMechanism(
            MechanismRuntimeConfig<? extends FlywheelMechanism> runtimeConfig,
            edu.wpi.first.math.controller.SimpleMotorFeedforward feedforward,
            MechanismLifecycleHooks lifecycleHooks) {
        super(runtimeConfig, feedforward, lifecycleHooks);
    }

    public static class StatefulFlywheelMechanism<E>
            extends FlywheelMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulFlywheelMechanism<E>, E> stateMachineCore;

        StatefulFlywheelMechanism(
                MechanismRuntimeConfig<? extends StatefulFlywheelMechanism<E>> runtimeConfig,
                E initialState,
                StatefulMechanismRuntimeConfig<StatefulFlywheelMechanism<E>, E> stateRuntimeConfig,
                SimpleMotorFeedforward feedforward,
                MechanismLifecycleHooks lifecycleHooks) {
            super(
                    runtimeConfig,
                    feedforward,
                    lifecycleHooks);
            stateMachineCore = StatefulMechanismCore.fromRuntimeConfig(
                    initialState,
                    this::atSetpoint,
                    stateRuntimeConfig);
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
