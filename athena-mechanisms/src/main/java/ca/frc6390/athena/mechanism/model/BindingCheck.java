package ca.frc6390.athena.mechanism.model;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Debounced boolean check over a measured quantity, used by {@code ControlBinding} checks such as
 * {@code blocked()} and {@code feedbackStalled()}.
 *
 * <p>The check is true once the metric has stayed past its threshold for {@link #forSeconds(double)}.
 * It is false the moment the metric falls back inside the threshold. A NaN metric is treated as
 * not met. Both {@link #threshold(double)} and {@link #forSeconds(double)} return new checks.</p>
 */
public final class BindingCheck implements BooleanSupplier {
    private final DoubleSupplier metric;
    private final double threshold;
    private final boolean above;
    private final double seconds;
    private final DoubleSupplier clock;
    private double conditionSince = Double.NaN;

    public BindingCheck(DoubleSupplier metric, double threshold, boolean above, double seconds,
            DoubleSupplier clock) {
        this.metric = Objects.requireNonNull(metric, "metric");
        this.threshold = threshold;
        this.above = above;
        this.seconds = Math.max(0.0, seconds);
        this.clock = Objects.requireNonNull(clock, "clock");
    }

    /** Returns the same check with a different threshold. */
    public BindingCheck threshold(double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Check threshold must be finite.");
        }
        return new BindingCheck(metric, value, above, seconds, clock);
    }

    /** Returns the same check requiring the condition to hold for a duration. */
    public BindingCheck forSeconds(double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException("Check duration must be finite and non-negative.");
        }
        return new BindingCheck(metric, threshold, above, value, clock);
    }

    /** Returns the current metric value. */
    public double value() {
        return metric.getAsDouble();
    }

    @Override
    public synchronized boolean getAsBoolean() {
        double value = metric.getAsDouble();
        boolean condition = Double.isFinite(value) && (above ? value > threshold : value < threshold);
        if (!condition) {
            conditionSince = Double.NaN;
            return false;
        }
        double now = clock.getAsDouble();
        if (Double.isNaN(conditionSince)) {
            conditionSince = now;
        }
        return now - conditionSince >= seconds;
    }
}
