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

        private final StatefulMechanismCore<StatefulFlywheelMechanism<E>, E> stateCore;

        public StatefulFlywheelMechanism(MechanismConfig<StatefulFlywheelMechanism<E>> config,
                                         SimpleMotorFeedforward feedforward,
                                         OutputType feedforwardOutputType,
                                         E initialState) {
            super(config, feedforward, feedforwardOutputType);
            stateCore = StatefulMechanismCore.fromConfig(initialState, this::atSetpoint, config);
        }

        @Override
        public StatefulMechanismCore<StatefulFlywheelMechanism<E>, E> stateCore() {
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
        public StatefulFlywheelMechanism<E> publishNetworkTables(String ownerHint) {
            super.publishNetworkTables(ownerHint);
            return this;
        }
    }

    @Override
    protected double getPidMeasurement() {
        return getVelocity();
    }
}
