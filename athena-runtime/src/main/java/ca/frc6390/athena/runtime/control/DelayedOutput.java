package ca.frc6390.athena.runtime.control;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Boolean gate that requires input to remain true for a delay window.
 */
public final class DelayedOutput implements BooleanSupplier {
    private final Debouncer debouncer;

    /**
     * Creates delayed output with a system clock.
     *
     * @param input boolean input
     * @param delaySeconds delay window
     */
    public DelayedOutput(BooleanSupplier input, double delaySeconds) {
        debouncer = new Debouncer(input, delaySeconds);
    }

    /**
     * Creates delayed output with an explicit clock.
     *
     * @param input boolean input
     * @param delaySeconds delay window
     * @param clockSeconds clock in seconds
     */
    public DelayedOutput(BooleanSupplier input, double delaySeconds, DoubleSupplier clockSeconds) {
        debouncer = new Debouncer(input, delaySeconds, clockSeconds);
    }

    @Override
    public boolean getAsBoolean() {
        return debouncer.getAsBoolean();
    }
}
