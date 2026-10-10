package ca.frc6390.athena.mechanism.model;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;

/**
 * Decides whether normal-operation data is informative enough to update the running estimate.
 *
 * <p>A sample is offered with its regressor row and its measured velocity and acceleration. The gate
 * keeps a sliding window of the most recent {@link ExcitationCriteria#windowSize() windowSize} rows
 * and accepts a sample only when every criterion holds on that window:</p>
 * <ol>
 *   <li>at least {@link ExcitationCriteria#minSamples() minSamples} samples have been offered since
 *       the last reset (the window must also be full);</li>
 *   <li>the standard deviation of velocity is at least {@code minVelocityStdDev};</li>
 *   <li>the standard deviation of acceleration is at least {@code minAccelerationStdDev};</li>
 *   <li>the 1-norm condition number of the window's equilibrated regressors is at most
 *       {@code maxConditionNumber}. A constant-velocity window fails this, because its columns are
 *       collinear.</li>
 * </ol>
 */
public final class ExcitationGate {

    /**
     * Thresholds for the excitation gate. Velocity is in base units per second, acceleration in base
     * units per second squared.
     *
     * @param minSamples samples required before any update
     * @param windowSize sliding window length used for the variance and conditioning checks
     * @param minVelocityStdDev minimum velocity standard deviation over the window
     * @param minAccelerationStdDev minimum acceleration standard deviation over the window
     * @param maxConditionNumber maximum condition number of the window's regressors
     */
    public record ExcitationCriteria(
            int minSamples,
            int windowSize,
            double minVelocityStdDev,
            double minAccelerationStdDev,
            double maxConditionNumber) {
        public static final ExcitationCriteria DEFAULT =
                new ExcitationCriteria(200, 100, 0.05, 0.05, 1.0e4);

        public ExcitationCriteria {
            if (minSamples <= 0 || windowSize < 3) {
                throw new IllegalArgumentException("Excitation sample counts must be positive (window at least 3).");
            }
            if (!(minVelocityStdDev >= 0.0) || !(minAccelerationStdDev >= 0.0)
                    || !(maxConditionNumber > 1.0)) {
                throw new IllegalArgumentException("Excitation thresholds must be non-negative.");
            }
        }
    }

    /**
     * Result of offering one sample.
     *
     * @param accepted whether the sample should update the estimator
     * @param reason human-readable reason, used in telemetry
     */
    public record Decision(boolean accepted, String reason) {
    }

    private record Row(double[] regressor, double velocity, double acceleration) {
    }

    private final ExcitationCriteria criteria;
    private final Deque<Row> window = new ArrayDeque<>();
    private long offered;

    public ExcitationGate(ExcitationCriteria criteria) {
        this.criteria = Objects.requireNonNull(criteria, "criteria");
    }

    public ExcitationCriteria criteria() {
        return criteria;
    }

    /** Offers one sample and returns whether it passes the excitation criteria. */
    public Decision offer(double[] regressor, double velocity, double acceleration) {
        Objects.requireNonNull(regressor, "regressor");
        offered++;
        window.addLast(new Row(regressor.clone(), velocity, acceleration));
        while (window.size() > criteria.windowSize()) {
            window.removeFirst();
        }
        return assess();
    }

    /** Clears the window and the sample count. */
    public void reset() {
        window.clear();
        offered = 0L;
    }

    /** Returns the number of samples offered since the last reset. */
    public long offered() {
        return offered;
    }

    private Decision assess() {
        if (offered < criteria.minSamples()) {
            return new Decision(false, "need " + criteria.minSamples() + " samples, have " + offered);
        }
        if (window.size() < criteria.windowSize()) {
            return new Decision(false, "window filling " + window.size() + "/" + criteria.windowSize());
        }
        double velocityStd = stdDev(row -> row.velocity());
        if (!(velocityStd >= criteria.minVelocityStdDev())) {
            return new Decision(false, "velocity variance too low (std " + format(velocityStd) + ")");
        }
        double accelerationStd = stdDev(row -> row.acceleration());
        if (!(accelerationStd >= criteria.minAccelerationStdDev())) {
            return new Decision(false, "acceleration variance too low (std " + format(accelerationStd) + ")");
        }
        double[][] rows = new double[window.size()][];
        int index = 0;
        for (Row row : window) {
            rows[index++] = row.regressor();
        }
        double condition = LeastSquares.conditionNumber(rows);
        if (!(condition <= criteria.maxConditionNumber())) {
            return new Decision(false, "regressors ill-conditioned (condition " + format(condition) + ")");
        }
        return new Decision(true, "excited");
    }

    private double stdDev(java.util.function.ToDoubleFunction<Row> value) {
        double mean = 0.0;
        for (Row row : window) {
            mean += value.applyAsDouble(row);
        }
        mean /= window.size();
        double sum = 0.0;
        for (Row row : window) {
            double delta = value.applyAsDouble(row) - mean;
            sum += delta * delta;
        }
        return Math.sqrt(sum / window.size());
    }

    private static String format(double value) {
        return String.format(java.util.Locale.ROOT, "%.3g", value);
    }
}
