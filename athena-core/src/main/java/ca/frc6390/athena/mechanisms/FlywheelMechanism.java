package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Simple motor mechanism intended for flywheel shooters and rollers.
 */
public class FlywheelMechanism extends SimpleMotorMechanism {

    private final boolean useVelocityPid;

    public FlywheelMechanism(MechanismConfig<? extends FlywheelMechanism> config, SimpleMotorFeedforward feedforward) {
        super(config, feedforward);
        this.useVelocityPid = config.data().pidUseVelocity();
    }

    public static class StatefulFlywheelMechanism<E extends Enum<E> & SetpointProvider<Double>>
            extends FlywheelMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulFlywheelMechanism<E>, E> stateCore;

        public StatefulFlywheelMechanism(MechanismConfig<StatefulFlywheelMechanism<E>> config,
                                         SimpleMotorFeedforward feedforward,
                                         E initialState) {
            super(config, feedforward);
            stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.data().stateMachineDelay(),
                    config.stateActions, config.stateHooks, config.exitStateHooks, config.alwaysHooks, config.exitAlwaysHooks,
                    config.inputs, config.doubleInputs, config.objectInputs);
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
        public StatefulFlywheelMechanism<E> publishNetworkTables(String ownerHint) {
            super.publishNetworkTables(ownerHint);
            return this;
        }
    }

    @Override
    protected double getPidMeasurement() {
        return useVelocityPid ? getVelocity() : super.getPidMeasurement();
    }
}
