package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Simple motor mechanism intended for flywheel shooters and rollers.
 */
public class FlywheelMechanism extends SimpleMotorMechanism {

    public FlywheelMechanism(MechanismConfig<? extends FlywheelMechanism> config,
                             SimpleMotorFeedforward feedforward,
                             OutputType feedforwardOutputType) {
        super(config, feedforward, feedforwardOutputType);
    }

    public static class StatefulFlywheelMechanism<E extends Enum<E> & SetpointProvider<Double>>
            extends FlywheelMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulFlywheelMechanism<E>, E> stateMachineCore;

        public StatefulFlywheelMechanism(MechanismConfig<StatefulFlywheelMechanism<E>> config,
                                         SimpleMotorFeedforward feedforward,
                                         OutputType feedforwardOutputType,
                                         E initialState) {
            super(config, feedforward, feedforwardOutputType);
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

    @Override
    protected double getPidMeasurement() {
        return velocity();
    }
}
