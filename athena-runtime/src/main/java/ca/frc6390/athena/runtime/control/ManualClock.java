package ca.frc6390.athena.runtime.control;

import java.util.function.DoubleSupplier;

/**
 * Mutable clock for deterministic tests and examples.
 */
public final class ManualClock implements DoubleSupplier {
    private double seconds;

    /**
     * Advances this clock.
     *
     * @param deltaSeconds seconds to add
     * @return this clock
     */
    public ManualClock advance(double deltaSeconds) {
        if (Double.isFinite(deltaSeconds) && deltaSeconds > 0.0) {
            seconds += deltaSeconds;
        }
        return this;
    }

    @Override
    public double getAsDouble() {
        return seconds;
    }
}
