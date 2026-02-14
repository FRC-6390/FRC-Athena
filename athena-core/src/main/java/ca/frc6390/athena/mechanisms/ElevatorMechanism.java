package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.controllers.ElevatorFeedForwardsSendable;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorMechanism extends Mechanism {

    private final ElevatorFeedForwardsSendable feedforward;
    private final OutputType feedforwardOutputType;

    public ElevatorMechanism(MechanismConfig<? extends ElevatorMechanism> config,
                             ElevatorFeedforward feedforward,
                             OutputType feedforwardOutputType) {
        super(ElevatorMechanismVisualization.prepare(config));
        if (feedforward != null) {
            this.feedforward = new ElevatorFeedForwardsSendable(feedforward.getKs(),feedforward.getKg(),feedforward.getKv(),feedforward.getKa());
            this.feedforwardOutputType = feedforwardOutputType != null ? feedforwardOutputType : OutputType.VOLTAGE;
            setFeedforwardEnabled(true);
        } else {
            this.feedforward = null;
            this.feedforwardOutputType = null;
            setFeedforwardEnabled(false);
        }
        if (config.elevatorSimulationParameters() != null) {
            MechanismConfig.ElevatorSimulationParameters params = config.elevatorSimulationParameters();
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
        if (feedforward == null) {
            return 0.0;
        }
        double valueVolts = feedforward.calculate(getControllerSetpointVelocity());
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

    public ElevatorMechanism publishNetworkTables(String ownerHint) {
        super.publishNetworkTables(ownerHint);
        return this;
    }
    
    public static class StatefulElevatorMechanism<E extends Enum<E> & StateMachine.SetpointProvider<Double>> extends ElevatorMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulElevatorMechanism<E>, E> stateCore;

        public StatefulElevatorMechanism(MechanismConfig<StatefulElevatorMechanism<E>> config,
                                         ElevatorFeedforward feedforward,
                                         OutputType feedforwardOutputType,
                                         E initialState) {
            super(config, feedforward, feedforwardOutputType);
            stateCore = StatefulMechanismCore.fromConfig(initialState, this::atSetpoint, config);
        }

        @Override
        public StatefulMechanismCore<StatefulElevatorMechanism<E>, E> stateCore() {
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
        public StatefulElevatorMechanism<E> publishNetworkTables(String ownerHint) {
            super.publishNetworkTables(ownerHint);
            return this;
        }
    }

}
