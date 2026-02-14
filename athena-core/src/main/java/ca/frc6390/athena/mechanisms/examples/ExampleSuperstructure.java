package ca.frc6390.athena.mechanisms.examples;

import java.util.function.BooleanSupplier;

import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.StateGraph;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;

/**
 * Minimal example that demonstrates how a superstructure can declare safe travel paths between
 * states using {@link StateGraph}. The guards reference conditions supplied by cooperating
 * mechanisms (arm, elevator, wrist) so callers only request the desired goal state while the graph
 * injects required transition states.
 */
public final class ExampleSuperstructure {

    private ExampleSuperstructure() {}

    /**
     * Mock state enum that would typically live alongside a team's real superstructure implementation.
     */
    public enum SuperStructureState implements SetpointProvider<Double> {
        STOWED(0.0),
        CLEARANCE(0.35),
        SCORE_HIGH(1.0);

        private final double setpoint;

        SuperStructureState(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    /**
     * Builds a {@link MechanismConfig} that wires the example superstructure up with a guarded state
     * graph. Consumers can call {@link MechanismConfig#build()} to obtain a ready-to-use
     * {@link StatefulMechanism} whose internal state machine already knows about the transition
     * requirements.
     *
     * @param armReady condition that reports when the arm is clear of the frame perimeter
     * @param elevatorReady condition that reports when the elevator has cleared its intermediate setpoint
     * @param wristReady condition that reports when the wrist has reached its extension angle
     * @return configured mechanism builder for the example superstructure
     */
    public static MechanismConfig<StatefulMechanism<SuperStructureState>> createConfig(
            BooleanSupplier armReady,
            BooleanSupplier elevatorReady,
            BooleanSupplier wristReady) {

        return MechanismConfig
                .statefulGeneric(SuperStructureState.STOWED)
                .stateMachineDelay(0.05)
                .stateGraph(buildGraph(armReady, elevatorReady, wristReady));
    }

    /**
     * Applies the same guarded graph to an already constructed superstructure. Useful when the
     * mechanism was instantiated elsewhere (for example, through dependency injection).
     *
     * @param superStructure stateful mechanism that coordinates multiple subsystems
     * @param armReady condition that reports when the arm is clear of the frame perimeter
     * @param elevatorReady condition that reports when the elevator has cleared its intermediate setpoint
     * @param wristReady condition that reports when the wrist has reached its extension angle
     */
    public static void attachGraph(
            StatefulMechanism<SuperStructureState> superStructure,
            BooleanSupplier armReady,
            BooleanSupplier elevatorReady,
            BooleanSupplier wristReady) {

        superStructure.setStateGraph(buildGraph(armReady, elevatorReady, wristReady));
    }

    private static StateGraph<SuperStructureState> buildGraph(
            BooleanSupplier armReady,
            BooleanSupplier elevatorReady,
            BooleanSupplier wristReady) {

        BooleanSupplier armAndElevatorClear = StateGraph.Guards.allOf(armReady, elevatorReady);

        return StateGraph
                .create(SuperStructureState.class)
                .path(SuperStructureState.STOWED, SuperStructureState.CLEARANCE, SuperStructureState.SCORE_HIGH)
                .guardPath(armAndElevatorClear, SuperStructureState.STOWED, SuperStructureState.CLEARANCE)
                .guard(SuperStructureState.CLEARANCE, SuperStructureState.SCORE_HIGH,
                        StateGraph.Guards.allOf(armAndElevatorClear, wristReady));
    }
}
