package ca.frc6390.athena.hardware.power;

/**
 * Online estimate of the battery open-circuit voltage and internal resistance from the model
 * {@code V_bus = Voc - I * R}.
 *
 * <p>Uses exponentially weighted means and covariances of bus voltage and total battery current
 * (a recursive least-squares line fit with forgetting). The slope {@code -R = cov(I, V) / var(I)} is
 * only updated when the current has enough variation (excitation), so a steady load does not
 * corrupt the estimate. The result is clamped to {@code [MIN_OHMS, MAX_OHMS]}, and the open-circuit
 * voltage is {@code Voc = meanV + R * meanI}.</p>
 */
public final class BatteryResistanceEstimator {
    /** Lower clamp on the estimate in ohms. */
    public static final double MIN_OHMS = 0.001;
    /** Upper clamp on the estimate in ohms. */
    public static final double MAX_OHMS = 0.5;
    /** Minimum current variance in A^2 before the slope is updated (about 0.5 A standard deviation). */
    public static final double MIN_CURRENT_VARIANCE = 0.25;

    private final double alpha;
    private final double initialOhms;
    private int samples;
    private double meanCurrent;
    private double meanVoltage;
    private double varianceCurrent;
    private double covariance;
    private double ohms;

    /**
     * Creates an estimator.
     *
     * @param forgettingTimeSeconds time constant of the exponential forgetting
     * @param cycleSeconds nominal sample period
     * @param initialOhms resistance used until enough excitation has been observed
     */
    public BatteryResistanceEstimator(double forgettingTimeSeconds, double cycleSeconds, double initialOhms) {
        if (!(forgettingTimeSeconds > 0.0) || !(cycleSeconds > 0.0)) {
            throw new IllegalArgumentException("Estimator time constants must be positive.");
        }
        this.alpha = 1.0 - Math.exp(-cycleSeconds / forgettingTimeSeconds);
        this.initialOhms = clamp(initialOhms);
        this.ohms = this.initialOhms;
    }

    /**
     * Adds one sample.
     *
     * @param busVolts measured bus voltage; ignored when not finite
     * @param totalAmps total battery current at the same instant; ignored when not finite
     */
    public void sample(double busVolts, double totalAmps) {
        if (!Double.isFinite(busVolts) || !Double.isFinite(totalAmps)) {
            return;
        }
        if (samples == 0) {
            meanCurrent = totalAmps;
            meanVoltage = busVolts;
            samples++;
            return;
        }
        double dI = totalAmps - meanCurrent;
        double dV = busVolts - meanVoltage;
        meanCurrent += alpha * dI;
        meanVoltage += alpha * dV;
        varianceCurrent = (1.0 - alpha) * (varianceCurrent + alpha * dI * dI);
        covariance = (1.0 - alpha) * (covariance + alpha * dI * dV);
        samples++;
        if (varianceCurrent >= MIN_CURRENT_VARIANCE) {
            ohms = clamp(-covariance / varianceCurrent);
        }
    }

    /** Returns the current internal-resistance estimate in ohms. */
    public double ohms() { return ohms; }

    /** Returns the open-circuit voltage estimate, or NaN before the first sample. */
    public double openCircuitVolts() {
        return samples == 0 ? Double.NaN : meanVoltage + ohms * meanCurrent;
    }

    /** Returns whether the resistance has been updated from observed excitation. */
    public boolean excited() { return varianceCurrent >= MIN_CURRENT_VARIANCE; }

    /** Returns the number of samples seen. */
    public int samples() { return samples; }

    /** Returns the initial resistance used before excitation. */
    public double initialOhms() { return initialOhms; }

    private static double clamp(double value) {
        if (!Double.isFinite(value)) return PowerPolicy.DEFAULT_BATTERY_OHMS;
        return Math.max(MIN_OHMS, Math.min(MAX_OHMS, value));
    }
}
