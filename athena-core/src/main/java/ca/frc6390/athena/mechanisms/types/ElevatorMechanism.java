package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.arcp.ARCP;
import ca.frc6390.athena.mechanisms.statespec.StateSpecAccess;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorMechanism extends Mechanism {

    private final ElevatorFeedforward feedforward;

    protected ElevatorMechanism(
            MechanismRuntimeConfig<? extends ElevatorMechanism> runtimeConfig,
            ElevatorFeedforward feedforward,
            double minHeightMeters,
            double maxHeightMeters,
            double unitsPerMeter,
            MechanismLifecycleHooks lifecycleHooks) {
        super(
                runtimeConfig,
                runtimeConfig.sourceKey() != null ? runtimeConfig.sourceKey() : runtimeConfig,
                lifecycleHooks);
        if (feedforward != null) {
            this.feedforward = feedforward;
            control().feedforwardEnabled(true);
        } else {
            this.feedforward = null;
        }
        applyKnownTravelRange(minHeightMeters, maxHeightMeters, unitsPerMeter);
    }

    private void applyKnownTravelRange(double minHeightMeters, double maxHeightMeters, double unitsPerMeter) {
        double min = minHeightMeters;
        double max = maxHeightMeters;
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

    public static class StatefulElevatorMechanism<E> extends ElevatorMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulElevatorMechanism<E>, E> stateMachineCore;

        StatefulElevatorMechanism(
                MechanismRuntimeConfig<? extends StatefulElevatorMechanism<E>> runtimeConfig,
                E initialState,
                StatefulMechanismRuntimeConfig<StatefulElevatorMechanism<E>, E> stateRuntimeConfig,
                ElevatorFeedforward feedforward,
                double minHeightMeters,
                double maxHeightMeters,
                double unitsPerMeter,
                MechanismLifecycleHooks lifecycleHooks) {
            super(
                    runtimeConfig,
                    feedforward,
                    minHeightMeters,
                    maxHeightMeters,
                    unitsPerMeter,
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
