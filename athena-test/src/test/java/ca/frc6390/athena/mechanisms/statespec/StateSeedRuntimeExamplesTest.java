package ca.frc6390.athena.mechanisms.statespec;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import ca.frc6390.athena.mechanisms.examples.state.StateSeedExamples;
import org.junit.jupiter.api.Test;

final class StateSeedRuntimeExamplesTest {

    @Test
    void resolvesAutoSetpointAsNull() {
        assertNull(StateSeedExamples.resolvedSetpoint(StateSeedExamples.SeededState.IDLE));
    }

    @Test
    void resolvesExplicitSetpointSeed() {
        assertEquals(1.2, StateSeedExamples.resolvedSetpoint(StateSeedExamples.SeededState.HOLD), 1e-9);
    }

    @Test
    void resolvesDslSetpointSeed() {
        assertEquals(2.75, StateSeedExamples.resolvedSetpoint(StateSeedExamples.SeededState.TRACK), 1e-9);
    }
}
