package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.controllers.ElevatorFeedForwardsSendable;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ElevatorMechanism extends Mechanism {

    private final ElevatorFeedForwardsSendable feedforward;

    public ElevatorMechanism(MechanismConfig<? extends ElevatorMechanism> config, ElevatorFeedforward feedforward) {
        super(ElevatorMechanismVisualization.prepare(config));
        this.feedforward = new ElevatorFeedForwardsSendable(feedforward.getKs(),feedforward.getKg(),feedforward.getKv(),feedforward.getKa());
        setFeedforwardEnabled(true);
        if (config.elevatorSimulationParameters != null) {
            MechanismConfig.ElevatorSimulationParameters params = config.elevatorSimulationParameters;
            double min = params.minHeightMeters;
            double max = params.maxHeightMeters;
            double unitsPerMeter = params.unitsPerMeterOverride;
            if (Double.isFinite(unitsPerMeter) && Math.abs(unitsPerMeter) > 1e-9) {
                min *= unitsPerMeter;
                max *= unitsPerMeter;
            } else {
                min = Double.NaN;
                max = Double.NaN;
            }
            if (Double.isFinite(min) && Double.isFinite(max) && max != min) {
                if (max < min) {
                    double tmp = min;
                    min = max;
                    max = tmp;
                }
                MechanismTravelRange.registerKnownRange(this, min, max);
            }
        }
    }

    @Override
    public double calculateFeedForward() {
        double value = feedforward.calculate(getControllerSetpointVelocity());
        return  isUseVoltage() ? value : value / 12d;
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        tab.add("FeedForwards", feedforward);
        return super.shuffleboard(tab, level);
    }

    @Override
    public ElevatorMechanism shuffleboard(String tab, SendableLevel level) {
        return (ElevatorMechanism) super.shuffleboard(tab, level);
    }
    
    public static class StatefulElevatorMechanism<E extends Enum<E> & StateMachine.SetpointProvider<Double>> extends ElevatorMechanism {

        private final StatefulMechanismCore<StatefulElevatorMechanism<E>, E> stateCore;

        public StatefulElevatorMechanism(MechanismConfig<StatefulElevatorMechanism<E>> config, ElevatorFeedforward feedforward, E initialState) {
            super(config, feedforward);
            stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.stateMachineDelay, config.stateActions);
        }

        @Override
        public double getSetpoint() {
            return stateCore.getSetpoint();
        }

        @Override
        public void update() {
            setSuppressMotorOutput(!stateCore.update(this));
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
        public StatefulElevatorMechanism<E> shuffleboard(String tab, SendableLevel level) {
            return (StatefulElevatorMechanism<E>) super.shuffleboard(tab, level);
        }
    }

}
