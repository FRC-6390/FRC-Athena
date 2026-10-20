package ca.frc6390.athena.hardware.backend;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class OpenLoopCommandFilterTest {
    @Test
    void repeatedValuesAreSkippedAndChangesAreSent() {
        OpenLoopCommandFilter filter = new OpenLoopCommandFilter();

        assertTrue(filter.voltage(0.0));
        assertFalse(filter.voltage(0.0));
        assertTrue(filter.voltage(1.5));
        assertFalse(filter.voltage(1.5));
        assertTrue(filter.percent(1.5), "a mode change always sends");
        assertTrue(filter.voltage(1.5));
        assertTrue(filter.voltage(-0.0), "negative zero is a different command bit pattern");
    }

    @Test
    void invalidateForcesTheNextCommand() {
        OpenLoopCommandFilter filter = new OpenLoopCommandFilter();
        assertTrue(filter.percent(0.25));
        filter.invalidate();
        assertTrue(filter.percent(0.25));
    }

    @Test
    void aHeldValueIsResentPeriodically() {
        OpenLoopCommandFilter filter = new OpenLoopCommandFilter();
        assertTrue(filter.voltage(3.0));
        for (int i = 0; i < OpenLoopCommandFilter.MAX_SKIPPED; i++) {
            assertFalse(filter.voltage(3.0));
        }
        assertTrue(filter.voltage(3.0));
        assertFalse(filter.voltage(3.0));
    }
}
