package ca.frc6390.athena.mechanism.model;

import java.util.function.DoubleSupplier;

/**
 * Acceleration from successive velocity measurements. Updates are ignored until at least
 * {@code minimumIntervalSeconds} has passed, so several reads in one cycle return the same value.
 */
public final class AccelerationEstimator {
    private static final double MINIMUM_INTERVAL = 0.001;
    private static final double SMOOTHING = 0.5;

    private final DoubleSupplier clock;
    private double lastTime = Double.NaN;
    private double lastVelocity;
    private double acceleration;
    private boolean primed;

    public AccelerationEstimator(DoubleSupplier clock) {
        this.clock = java.util.Objects.requireNonNull(clock, "clock");
    }

    /** Returns the smoothed acceleration for the current velocity measurement. */
    public synchronized double update(double velocity) {
        double now = clock.getAsDouble();
        if (Double.isNaN(lastTime)) {
            lastTime = now;
            lastVelocity = velocity;
            return 0.0;
        }
        double dt = now - lastTime;
        if (dt < MINIMUM_INTERVAL) {
            return acceleration;
        }
        double raw = (velocity - lastVelocity) / dt;
        acceleration = primed ? SMOOTHING * raw + (1.0 - SMOOTHING) * acceleration : raw;
        primed = true;
        lastTime = now;
        lastVelocity = velocity;
        return acceleration;
    }

    public synchronized void reset() {
        lastTime = Double.NaN;
        acceleration = 0.0;
        primed = false;
    }
}
