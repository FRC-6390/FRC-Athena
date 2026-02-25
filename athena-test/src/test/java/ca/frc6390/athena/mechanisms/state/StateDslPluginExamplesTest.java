package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.mechanisms.examples.state.StateDslPluginExamples;
import ca.frc6390.athena.mechanisms.examples.state.StateDslPluginExamples.PluginState;

final class StateDslPluginExamplesTest {

    @Test
    void resolvesPluginStyleSeedSetpoints() {
        assertNull(StateDslPluginExamples.resolvedSetpoint(PluginState.STOW));
        assertEquals(0.2, StateDslPluginExamples.resolvedSetpoint(PluginState.LATCH), 1e-9);
        assertEquals(0.45, StateDslPluginExamples.resolvedSetpoint(PluginState.FEED), 1e-9);
        assertEquals(-0.25, StateDslPluginExamples.resolvedSetpoint(PluginState.UNJAM), 1e-9);
    }

    @Test
    void guardedGraphBlocksScorePathUntilFeedAllowed() {
        AtomicBoolean feedAllowed = new AtomicBoolean(false);
        StateMachine<Double, PluginState> machine = StateDslPluginExamples.createMachine(
                () -> true,
                feedAllowed::get,
                () -> true);

        StateDslPluginExamples.forceScore(machine);
        machine.update();
        assertEquals(PluginState.STOW, machine.getGoalState());

        feedAllowed.set(true);
        machine.update();
        assertEquals(PluginState.FEED, machine.getGoalState());
        assertEquals("LATCH, SCORE", machine.getNextStateQueue());
    }

    @Test
    void dslSeedQueuesNextStateTransition() {
        StateMachine<Double, PluginState> machine = StateDslPluginExamples.createMachine(
                () -> true,
                () -> true,
                () -> true);

        StateDslPluginExamples.forceUnjam(machine);
        machine.update();
        assertEquals(PluginState.UNJAM, machine.getGoalState());
        assertEquals("FEED", machine.getNextStateQueue());

        machine.update();
        assertEquals(PluginState.FEED, machine.getGoalState());
        assertEquals("LATCH", machine.getNextStateQueue());
    }
}
