package ca.frc6390.athena.runtime.control;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Boolean gate that emits true only after input has remained true for a period.
 */
public final class Debouncer implements BooleanSupplier {
    private final BooleanSupplier input;
    private final DoubleSupplier clockSeconds;
    private final double periodSeconds;
    private double firstTrueSeconds = Double.NaN;

    /**
     * Creates a debouncer using a monotonic system clock.
     *
     * @param input boolean input
     * @param periodSeconds required true period
     */
    public Debouncer(BooleanSupplier input, double periodSeconds) {
        this(input, periodSeconds, () -> System.nanoTime() / 1_000_000_000.0);
    }

    /**
     * Creates a debouncer with an explicit clock.
     *
     * @param input boolean input
     * @param periodSeconds required true period
     * @param clockSeconds clock in seconds
     */
    public Debouncer(BooleanSupplier input, double periodSeconds, DoubleSupplier clockSeconds) {
        this.input = input == null ? () -> false : input;
        this.periodSeconds = Math.max(0.0, periodSeconds);
        this.clockSeconds = clockSeconds == null ? () -> 0.0 : clockSeconds;
    }

    @Override
    public boolean getAsBoolean() {
        if (!input.getAsBoolean()) {
            firstTrueSeconds = Double.NaN;
            return false;
        }
        double now = clockSeconds.getAsDouble();
        if (Double.isNaN(firstTrueSeconds)) {
            firstTrueSeconds = now;
        }
        return now - firstTrueSeconds >= periodSeconds;
    }
}
