package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.mechanisms.examples.StateMachineExamples;
import ca.frc6390.athena.mechanisms.examples.StateMachineExamples.IntakeState;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

final class StateMachineExamplesTest {

    @Test
    void graphExpansionMatchesExamplePath() {
        StateGraph<IntakeState> graph = StateMachineExamples.createIntakeGraph(() -> true);

        assertEquals(
                java.util.List.of(IntakeState.CLEARANCE, IntakeState.INTAKE),
                graph.expand(IntakeState.STOW, IntakeState.INTAKE));
        assertEquals(
                java.util.List.of(IntakeState.CLEARANCE, IntakeState.STOW),
                graph.expand(IntakeState.INTAKE, IntakeState.STOW));
    }

    @Test
    void guardBlocksUntilReadyThenTransitions() {
        AtomicBoolean clearanceReady = new AtomicBoolean(false);
        StateMachine<Double, IntakeState> machine =
                StateMachineExamples.createIntakeMachine(() -> true, clearanceReady::get);

        StateMachineExamples.forceIntake(machine);
        machine.update();
        assertEquals(IntakeState.STOW, machine.getGoalState());

        clearanceReady.set(true);
        machine.update();
        assertEquals(IntakeState.CLEARANCE, machine.getGoalState());

        machine.update();
        assertEquals(IntakeState.INTAKE, machine.getGoalState());
    }

    @Test
    void appendForceKeepsIntermediatePath() {
        StateMachine<Double, IntakeState> machine =
                StateMachineExamples.createIntakeMachine(() -> true, () -> true);

        StateMachineExamples.forceIntake(machine);
        machine.update();
        machine.update();

        StateMachineExamples.forceStow(machine, true);
        assertEquals("CLEARANCE, STOW", machine.getNextStateQueue());
    }
}
