package ca.frc6390.athena.runtime.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.Test;

class ControllerHelperTest {
    @Test
    void modifiedAxisAppliesDeadzoneSquaringAndInversion() {
        ModifiedAxis axis = new ModifiedAxis(() -> 0.5, 0.1)
                .squared(true)
                .inverted(true);

        double shaped = (0.5 - 0.1) / (1.0 - 0.1);

        assertEquals(-(shaped * shaped), axis.getAsDouble(), 1.0e-9);
    }

    @Test
    void debouncerRequiresTrueInputForPeriod() {
        AtomicBoolean input = new AtomicBoolean(true);
        ManualClock clock = new ManualClock();
        Debouncer debouncer = new Debouncer(input::get, 0.25, clock);

        assertFalse(debouncer.getAsBoolean());
        clock.advance(0.24);
        assertFalse(debouncer.getAsBoolean());
        clock.advance(0.01);
        assertTrue(debouncer.getAsBoolean());
        input.set(false);
        assertFalse(debouncer.getAsBoolean());
    }

    @Test
    void delayedOutputRespectsInputLevel() {
        ManualClock clock = new ManualClock();
        DelayedOutput delayed = new DelayedOutput(() -> true, 0.0, clock);
        DelayedOutput blocked = new DelayedOutput(() -> false, 0.0, clock);

        assertTrue(delayed.getAsBoolean());
        assertFalse(blocked.getAsBoolean());
    }
}
