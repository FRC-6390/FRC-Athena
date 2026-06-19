package ca.frc6390.athena.mechanisms.examples;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.api.superstructure.SuperstructureDefinitions;
import ca.frc6390.athena.api.superstructure.Superstructures;
import ca.frc6390.athena.mechanisms.StateGraph;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * End-to-end example showing how to express a superstructure (elevator + end effector)
 * using the V2 superstructure API. The superstates mirror the old implementation:
 * L1-L4, Align/Home, Intaking, Score, Algae variants, etc. Constraints enforce sequencing,
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
        AlgaeHigh(EndEffectorState.AlgaeHigh, ElevatorState.AlgaeHigh),
        AlgaeLow(EndEffectorState.AlgaeLow, ElevatorState.AlgaeLow),
        ScoreAlgae(EndEffectorState.Score, ElevatorState.L4),
        AlgaeSpit(EndEffectorState.Score),

        L4(EndEffectorState.L4, ElevatorState.L4),
        L3(EndEffectorState.L3, ElevatorState.L3),
        L2(EndEffectorState.L2, ElevatorState.L2),
        L1(EndEffectorState.L1, ElevatorState.L1),

        StartConfiguration(EndEffectorState.StartConfiguration, ElevatorState.HomeReset),
        Align(EndEffectorState.Home, ElevatorState.Aligning),
        Home(EndEffectorState.Home, ElevatorState.HomeReset),
        HomePID(EndEffectorState.Home, ElevatorState.HomePID),
        Stopped(EndEffectorState.Stop),
        Score(EndEffectorState.Score),
        Intaking(EndEffectorState.Intaking, ElevatorState.Intaking);

        private final SuperTuple tuple;
        SuperState(EndEffectorState eff) { this(eff, null); }
        SuperState(EndEffectorState eff, ElevatorState elev) {
            this.tuple = new SuperTuple(eff, elev);
        }
        @Override public SuperTuple getSetpoint() { return tuple; }
    }

    /**
     * Builds a fully-wired superstructure with constraints and sim attachments.
     * - Child mechanisms use setpoint-as-output for simplicity; replace with PID/FF as needed.
     * - StateGraphs on the children enforce straight-line travel order.
     * - Constraints on the superstructure align with the legacy sequencing (stow before intake, etc.).
     */
    public static SuperstructureMechanism<SuperState, SuperTuple> buildSuperstructure() {
        @SuppressWarnings("unchecked")
        StatefulMechanism<ElevatorState> elevator =
                (StatefulMechanism<ElevatorState>) MechanismDefinitions.build(
                        Mechanisms.stateful("elevator", ElevatorState.HomeReset).definition());
        elevator.stateMachine().graph(StateGraph.create(ElevatorState.class)
                .path(ElevatorState.HomeReset, ElevatorState.Intaking)
                .path(ElevatorState.Intaking, ElevatorState.L1, ElevatorState.L2, ElevatorState.L3, ElevatorState.L4)
                .path(ElevatorState.HomeReset, ElevatorState.HomePID));

        @SuppressWarnings("unchecked")
        StatefulMechanism<EndEffectorState> eff =
                (StatefulMechanism<EndEffectorState>) MechanismDefinitions.build(
                        Mechanisms.stateful("end-effector", EndEffectorState.Home).definition());
        eff.stateMachine().graph(StateGraph.create(EndEffectorState.class)
                .path(EndEffectorState.Home, EndEffectorState.Intaking)
                .path(EndEffectorState.Intaking, EndEffectorState.L1, EndEffectorState.L2, EndEffectorState.L3, EndEffectorState.L4));

        @SuppressWarnings("unchecked")
        SuperstructureMechanism<SuperState, SuperTuple> superstructure =
                (SuperstructureMechanism<SuperState, SuperTuple>) SuperstructureDefinitions.build(
                        Superstructures.<SuperState, SuperTuple>stateful("composite", SuperState.Home)
                                .mechanisms(m -> m
                                        .existing(elevator, SuperTuple::elev)
                                        .existing(eff, SuperTuple::eff))
                                .behavior(behavior -> behavior.constraints(c -> c
                                        .state(SuperState.Intaking,
                                                ctx -> ctx.mechanisms().generic(SuperTuple::elev).stateMachine().at(ElevatorState.Intaking)
                                                        || ctx.mechanisms().generic(SuperTuple::eff).stateMachine().at(EndEffectorState.Home),
                                                SuperState.Home)
                                        .state(SuperState.L3,
                                                ctx -> ctx.mechanisms().generic(SuperTuple::eff).stateMachine().at(EndEffectorState.Home, EndEffectorState.L3))
                                        .state(SuperState.L4,
                                                ctx -> ctx.mechanisms().generic(SuperTuple::eff).stateMachine().at(EndEffectorState.Home, EndEffectorState.L4))
                                        .state(SuperState.Score,
                                                ctx -> ctx.mechanisms().generic(SuperTuple::elev).stateMachine().atGoal()
                                                        && ctx.mechanisms().generic(SuperTuple::eff).stateMachine().atGoal())))
                                .definition());
        return superstructure;
    }
}
