package ca.frc6390.athena.controllers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.controllers.examples.ControllerHelperExamples;
import org.junit.jupiter.api.Test;

final class ControllerHelperExamplesTest {

    @Test
    void tunedAxisAppliesDeadzoneAndSquaring() {
        ModifiedAxis axis = ControllerHelperExamples.tunedAxis(() -> 0.5, 0.1, true, false);
        double expected = Math.copySign(Math.pow((0.5 - 0.1) / (1.0 - 0.1), 2.0), 0.5);
        assertEquals(expected, axis.getAsDouble(), 1e-9);
    }

    @Test
    void tunedAxisCanInvertOutput() {
        ModifiedAxis axis = ControllerHelperExamples.tunedAxis(() -> 0.5, 0.1, false, true);
        double expected = -((0.5 - 0.1) / (1.0 - 0.1));
        assertEquals(expected, axis.getAsDouble(), 1e-9);
    }

    @Test
    void debouncedPressReturnsTrueWhenInputHeldAndPeriodSatisfied() {
        Debouncer debouncer = ControllerHelperExamples.debouncedPress(() -> true, -0.1);
        assertTrue(debouncer.getAsBoolean());
    }

    @Test
    void delayedHoldRespectsInputLevel() {
        DelayedOutput delayedTrue = ControllerHelperExamples.delayedHold(() -> true, 0.0);
        DelayedOutput delayedFalse = ControllerHelperExamples.delayedHold(() -> false, 0.0);

        assertTrue(delayedTrue.getAsBoolean());
        assertFalse(delayedFalse.getAsBoolean());
    }
}
