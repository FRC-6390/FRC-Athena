package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.mechanisms.examples.state.StateDslExamples;
import ca.frc6390.athena.mechanisms.examples.state.StateDslExamples.DslState;

final class StateDslExamplesTest {

    @Test
    void resolvesDslSeedSetpoints() {
        assertNull(StateDslExamples.resolvedSetpoint(DslState.STOW));
        assertEquals(0.2, StateDslExamples.resolvedSetpoint(DslState.LATCH), 1e-9);
        assertEquals(0.45, StateDslExamples.resolvedSetpoint(DslState.FEED), 1e-9);
        assertEquals(-0.25, StateDslExamples.resolvedSetpoint(DslState.UNJAM), 1e-9);
    }

    @Test
    void guardedGraphBlocksScorePathUntilFeedAllowed() {
        AtomicBoolean feedAllowed = new AtomicBoolean(false);
        StateMachine<Double, DslState> machine = StateDslExamples.createMachine(
                () -> true,
                feedAllowed::get,
                () -> true);

        StateDslExamples.forceScore(machine);
        machine.update();
        assertEquals(DslState.STOW, machine.getGoalState());

        feedAllowed.set(true);
        machine.update();
        assertEquals(DslState.FEED, machine.getGoalState());
        assertEquals("LATCH, SCORE", machine.getNextStateQueue());
    }

    @Test
    void dslSeedQueuesNextStateTransition() {
        StateMachine<Double, DslState> machine = StateDslExamples.createMachine(
                () -> true,
                () -> true,
                () -> true);

        StateDslExamples.forceUnjam(machine);
        machine.update();
        assertEquals(DslState.UNJAM, machine.getGoalState());
        assertEquals("FEED", machine.getNextStateQueue());

        machine.update();
        assertEquals(DslState.FEED, machine.getGoalState());
        assertEquals("LATCH", machine.getNextStateQueue());
    }
}
