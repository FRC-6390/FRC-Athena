package ca.frc6390.athena.controllers.examples;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.controllers.Debouncer;
import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.controllers.ModifiedAxis;

/**
 * Example helpers for configuring controller-side input shaping and gating.
 */
public final class ControllerHelperExamples {
    private ControllerHelperExamples() {}

    public static ModifiedAxis tunedAxis(
            DoubleSupplier rawAxis,
            double deadzone,
            boolean squared,
            boolean inverted) {
        return new ModifiedAxis(rawAxis, deadzone)
                .setSquaring(squared)
                .setInverted(inverted);
    }

    public static Debouncer debouncedPress(BooleanSupplier input, double periodSeconds) {
        return new Debouncer(input, periodSeconds);
    }

    public static DelayedOutput delayedHold(BooleanSupplier input, double holdSeconds) {
        return new DelayedOutput(input, holdSeconds);
    }
}
