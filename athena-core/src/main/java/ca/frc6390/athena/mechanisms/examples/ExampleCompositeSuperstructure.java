package ca.frc6390.athena.mechanisms.examples;

import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.StateGraph;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureConfig;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * End-to-end example showing how to express a superstructure (elevator + end effector)
 * using the new SuperstructureConfig API. The superstates mirror the old-style class:
 * L1-L4, Align/Home, Intaking, Score, Algae variants, etc. Guards enforce sequencing,
 * and the end effector visualization is anchored to the elevator carriage for sim.
 *
 * This example is self-contained: it builds the child mechanisms, applies state graphs,
 * and returns a ready SuperstructureMechanism. Replace the placeholder gains/lengths with
 * your robot values when integrating.
 */
public final class ExampleCompositeSuperstructure {

    private ExampleCompositeSuperstructure() {}

    // ------------------------
    // Child mechanism states
    // ------------------------
    public enum ElevatorState implements SetpointProvider<Double> {
        HomeReset(Units.inchesToMeters(24)),
        HomePID(Units.inchesToMeters(24)),
        Aligning(Units.inchesToMeters(33)),
        Intaking(Units.inchesToMeters(24)),
        L1(Units.inchesToMeters(27)),
        L2(Units.inchesToMeters(35.5)),
        L3(Units.inchesToMeters(49.5)),
        L4(Units.inchesToMeters(76.23)),
        AlgaeLow(Units.inchesToMeters(32.545)),
        AlgaeHigh(Units.inchesToMeters(46.77));

        private final double setpoint;
        ElevatorState(double setpoint) { this.setpoint = setpoint; }
        @Override public Double getSetpoint() { return setpoint; }
    }

    public enum EndEffectorState implements SetpointProvider<Double> {
        Home(0.0),
        Intaking(0.2),
        L1(0.4),
        L2(0.6),
        L3(0.8),
        L4(1.0),
        Score(0.7),
        AlgaeLow(0.3),
        AlgaeHigh(0.6),
        Stop(0.0),
        StartConfiguration(0.0);

        private final double setpoint;
        EndEffectorState(double setpoint) { this.setpoint = setpoint; }
        @Override public Double getSetpoint() { return setpoint; }
    }

    // ------------------------
    // Superstructure states
    // ------------------------
    public record SuperTuple(EndEffectorState eff, ElevatorState elev) {}

    public enum SuperState implements SetpointProvider<SuperTuple> {
        AlgaeHigh(new SuperTuple(EndEffectorState.AlgaeHigh, ElevatorState.AlgaeHigh)),
        AlgaeLow(new SuperTuple(EndEffectorState.AlgaeLow, ElevatorState.AlgaeLow)),
        ScoreAlgae(new SuperTuple(EndEffectorState.Score, ElevatorState.L4)),
        AlgaeSpit(new SuperTuple(EndEffectorState.Score, null)),

        L4(new SuperTuple(EndEffectorState.L4, ElevatorState.L4)),
        L3(new SuperTuple(EndEffectorState.L3, ElevatorState.L3)),
        L2(new SuperTuple(EndEffectorState.L2, ElevatorState.L2)),
        L1(new SuperTuple(EndEffectorState.L1, ElevatorState.L1)),

        StartConfiguration(new SuperTuple(EndEffectorState.StartConfiguration, ElevatorState.HomeReset)),
        Align(new SuperTuple(EndEffectorState.Home, ElevatorState.Aligning)),
        Home(new SuperTuple(EndEffectorState.Home, ElevatorState.HomeReset)),
        HomePID(new SuperTuple(EndEffectorState.Home, ElevatorState.HomePID)),
        Stopped(new SuperTuple(EndEffectorState.Stop, null)),
        Score(new SuperTuple(EndEffectorState.Score, null)),
        Intaking(new SuperTuple(EndEffectorState.Intaking, ElevatorState.Intaking));

        private final SuperTuple tuple;
        SuperState(SuperTuple tuple) { this.tuple = tuple; }
        @Override public SuperTuple getSetpoint() { return tuple; }
    }

    /**
     * Builds a fully-wired superstructure with guards and sim attachments.
     * - Child mechanisms use setpoint-as-output for simplicity; replace with PID/FF as needed.
     * - StateGraphs on the children enforce straight-line travel order.
     * - Guards on the superstructure align with the legacy sequencing (stow before intake, etc.).
     */
    public static SuperstructureMechanism<SuperState, SuperTuple> buildSuperstructure() {
        // Elevator config (stub PID/FF; replace with tuned values + motors/encoders).
        MechanismConfig<StatefulMechanism<ElevatorState>> elevator =
                MechanismConfig.statefulGeneric(ElevatorState.HomeReset);
        elevator.useSetpointAsOutput = true;
        elevator.stateMachineDelay = 0.04;
        elevator.stateGraph = StateGraph.create(ElevatorState.class)
                .path(ElevatorState.HomeReset, ElevatorState.Intaking)
                .path(ElevatorState.Intaking, ElevatorState.L1, ElevatorState.L2, ElevatorState.L3, ElevatorState.L4)
                .path(ElevatorState.HomeReset, ElevatorState.HomePID);

        // End effector config (stub; replace with real arm/wrist mechanism configs if you have them).
        MechanismConfig<StatefulMechanism<EndEffectorState>> eff =
                MechanismConfig.statefulGeneric(EndEffectorState.Home);
        eff.useSetpointAsOutput = true;
        eff.stateMachineDelay = 0.02;
        eff.stateGraph = StateGraph.create(EndEffectorState.class)
                .path(EndEffectorState.Home, EndEffectorState.Intaking)
                .path(EndEffectorState.Intaking, EndEffectorState.L1, EndEffectorState.L2, EndEffectorState.L3, EndEffectorState.L4);

        // Superstructure config mirrors the legacy sequencing.
        SuperstructureConfig<SuperState, SuperTuple> config =
                SuperstructureConfig.<SuperState, SuperTuple>builder(SuperState.Home)
                .addMech(elevator, SuperTuple::elev)
                .addMech(eff, SuperTuple::eff)
                // Guards similar to old class:
                // - Stow before intaking from high states
                .addGuard(SuperState.Intaking,
                        ctx -> ctx.mechanism(SuperTuple::elev).getStateMachine().atState(ElevatorState.Intaking)
                                || ctx.mechanism(SuperTuple::eff).getStateMachine().atState(EndEffectorState.Home))
                // - Keep end effector home before lifting to high nodes
                .addGuard(SuperState.L3,
                        ctx -> ctx.mechanism(SuperTuple::eff).getStateMachine().atState(EndEffectorState.Home, EndEffectorState.L3))
                .addGuard(SuperState.L4,
                        ctx -> ctx.mechanism(SuperTuple::eff).getStateMachine().atState(EndEffectorState.Home, EndEffectorState.L4))
                // - Score waits for both children to be at goal
                .addGuard(SuperState.Score,
                        ctx -> ctx.mechanism(SuperTuple::elev).getStateMachine().atGoalState()
                                && ctx.mechanism(SuperTuple::eff).getStateMachine().atGoalState())
                .setSimulation(sim -> {
                    // Anchor end effector visualization to the elevator carriage height.
                    sim.attach(SuperTuple::eff, ctx -> {
                        var elevMech = ctx.mechanism(SuperTuple::elev);
                        double z = elevMech.getPosition();
                        return new Pose3d(0.0, 0.0, z, new Rotation3d());
                    });
                })
                .buildConfig();

        return config.build();
    }
}
