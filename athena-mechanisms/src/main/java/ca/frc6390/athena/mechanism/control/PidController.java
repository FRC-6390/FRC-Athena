package ca.frc6390.athena.mechanism.control;

import java.util.Objects;

/** Stateful PID calculation backed by live-editable {@link PidGains}. */
public final class PidController {
    private final PidGains gains;
    private double minimumInput;
    private double maximumInput;
    private double minimumOutput = Double.NEGATIVE_INFINITY;
    private double maximumOutput = Double.POSITIVE_INFINITY;
    private boolean continuous;
    private double integral;
    private double previousMeasurement;
    private boolean first = true;
    private double proportionalOutput;
    private double integralOutput;
    private double derivativeOutput;
    private double lastUnclamped;
    private double lastClamped;

    PidController(PidGains gains) {
        this.gains = Objects.requireNonNull(gains, "gains");
    }

    /** Uses the shortest error across the supplied wrapping input range. */
    public PidController continuous(double minimumInput, double maximumInput) {
        if (!Double.isFinite(minimumInput)
                || !Double.isFinite(maximumInput)
                || maximumInput <= minimumInput) {
            throw new IllegalArgumentException("Continuous PID input range must be finite and increasing.");
        }
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
        continuous = true;
        return this;
    }

    /** Clamps output and prevents the integral term from winding farther into saturation. */
    public PidController outputRange(double minimumOutput, double maximumOutput) {
        if (!Double.isFinite(minimumOutput)
                || !Double.isFinite(maximumOutput)
                || maximumOutput <= minimumOutput) {
            throw new IllegalArgumentException("PID output range must be finite and increasing.");
        }
        this.minimumOutput = minimumOutput;
        this.maximumOutput = maximumOutput;
        return this;
    }

    /** Clears integral and derivative history. */
    public void reset() {
        integral = 0.0;
        previousMeasurement = 0.0;
        proportionalOutput = 0.0;
        integralOutput = 0.0;
        derivativeOutput = 0.0;
        lastUnclamped = 0.0;
        lastClamped = 0.0;
        first = true;
    }

    /**
     * Forgets the previous measurement so the next derivative term is zero. Used on a gain change, where
     * the derivative of the old gains must not leak into the new gains' output.
     */
    public void resetDerivative() {
        first = true;
    }

    /**
     * Bumpless transfer. Re-seeds the integral so that this controller's unclamped output, plus the
     * contribution of every other loop ({@code currentVolts - clampedOutput}), equals {@code targetVolts}.
     * Returns false (and changes nothing) when the controller has no integral action.
     */
    public boolean bumpTo(double targetVolts, double currentVolts) {
        if (gains.i() == 0.0 || !Double.isFinite(targetVolts) || !Double.isFinite(currentVolts)) {
            return false;
        }
        double others = currentVolts - lastClamped;
        double shift = targetVolts - (lastUnclamped + others);
        integral += shift / gains.i();
        lastUnclamped += shift;
        integralOutput += shift;
        lastClamped = clamp(lastUnclamped);
        return true;
    }

    /** Calculates an output in the units implied by the configured gains. */
    public double calculate(double measurement, double target, double dtSeconds) {
        if (!Double.isFinite(measurement) || !Double.isFinite(target)) {
            throw new IllegalArgumentException("PID measurement and target must be finite.");
        }
        if (gains.isDisabled()) {
            return 0.0;
        }

        double error = wrap(target - measurement);
        double dt = Double.isFinite(dtSeconds) && dtSeconds > 0.0 ? dtSeconds : 0.0;
        double derivative = first || dt == 0.0
                ? 0.0
                : -wrap(measurement - previousMeasurement) / dt;
        if (dt > 0.0 && (gains.iZone() <= 0.0 || Math.abs(error) <= gains.iZone())) {
            integral = antiWindupIntegral(integral + error * dt, error, derivative);
        } else if (gains.iZone() > 0.0 && Math.abs(error) > gains.iZone()) {
            integral = 0.0;
        }
        first = false;
        previousMeasurement = measurement;
        proportionalOutput = gains.p() * error;
        integralOutput = gains.i() * integral;
        derivativeOutput = gains.d() * derivative;
        lastUnclamped = proportionalOutput + integralOutput + derivativeOutput;
        lastClamped = clamp(lastUnclamped);
        return lastClamped;
    }

    void applied(double requested, double applied) {
        if (gains.i() == 0.0 || !Double.isFinite(requested) || !Double.isFinite(applied)) {
            return;
        }
        double excess = requested - applied;
        double output = gains.i() * integral;
        if (excess > 0.0 && output > 0.0) {
            integral -= Math.min(excess, output) / gains.i();
        } else if (excess < 0.0 && output < 0.0) {
            integral -= Math.max(excess, output) / gains.i();
        }
    }

    double proportionalOutput() { return proportionalOutput; }
    double integralOutput() { return integralOutput; }
    double derivativeOutput() { return derivativeOutput; }

    private double antiWindupIntegral(double candidate, double error, double derivative) {
        if (gains.i() == 0.0) {
            return 0.0;
        }
        double nonIntegral = gains.p() * error + gains.d() * derivative;
        double current = nonIntegral + gains.i() * integral;
        double next = nonIntegral + gains.i() * candidate;
        double delta = gains.i() * (candidate - integral);
        if (next > maximumOutput && delta > 0.0) {
            return current < maximumOutput
                    ? integral + (maximumOutput - current) / gains.i()
                    : integral;
        }
        if (next < minimumOutput && delta < 0.0) {
            return current > minimumOutput
                    ? integral + (minimumOutput - current) / gains.i()
                    : integral;
        }
        return candidate;
    }

    private double clamp(double value) {
        return Math.max(minimumOutput, Math.min(maximumOutput, value));
    }

    private double wrap(double value) {
        if (!continuous) {
            return value;
        }
        // Errors and measurement deltas are differences, so they wrap into the symmetric
        // shortest-path interval [-span/2, span/2) regardless of where the input range starts.
        double modulus = maximumInput - minimumInput;
        double halfSpan = modulus / 2.0;
        double wrapped = (value + halfSpan) % modulus;
        if (wrapped < 0.0) {
            wrapped += modulus;
        }
        return wrapped - halfSpan;
    }
}
