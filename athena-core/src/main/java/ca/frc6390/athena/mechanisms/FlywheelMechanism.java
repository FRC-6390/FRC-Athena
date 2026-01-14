package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Simple motor mechanism intended for flywheel shooters and rollers.
 */
public class FlywheelMechanism extends SimpleMotorMechanism {

    private final boolean useVelocityPid;

    public FlywheelMechanism(MechanismConfig<? extends FlywheelMechanism> config, SimpleMotorFeedforward feedforward) {
        super(config, feedforward);
        this.useVelocityPid = config.pidUseVelocity;
    }

    @Override
    public FlywheelMechanism shuffleboard(String tab, SendableLevel level) {
        return (FlywheelMechanism) super.shuffleboard(tab, level);
    }

    public static class StatefulFlywheelMechanism<E extends Enum<E> & SetpointProvider<Double>>
            extends FlywheelMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulFlywheelMechanism<E>, E> stateCore;

        public StatefulFlywheelMechanism(MechanismConfig<StatefulFlywheelMechanism<E>> config,
                                         SimpleMotorFeedforward feedforward,
                                         E initialState) {
            super(config, feedforward);
            stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.stateMachineDelay,
                    config.stateActions, config.stateHooks, config.alwaysHooks, config.inputs, config.doubleInputs, config.objectInputs);
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
        public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
            stateCore.shuffleboard(tab, level);
            return super.shuffleboard(tab, level);
        }

        @SuppressWarnings("unchecked")
        @Override
        public StatefulFlywheelMechanism<E> shuffleboard(String tab, SendableLevel level) {
            return (StatefulFlywheelMechanism<E>) super.shuffleboard(tab, level);
        }
    }

    @Override
    protected double getPidMeasurement() {
        return useVelocityPid ? getVelocity() : super.getPidMeasurement();
    }
}
