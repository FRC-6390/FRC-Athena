package ca.frc6390.athena.wpilib.controls;

/** Allocation-free directional slew limiter for one normalized axis pipeline. */
final class AxisSlewLimiter {
    private double value;
    private double lastTimestamp;
    private boolean initialized;

    double calculate(double target, double timestamp, double positiveRate, double negativeRate) {
        double now = Double.isFinite(timestamp) ? timestamp : 0.0;
        double dt = initialized ? Math.max(0.0, Math.min(0.1, now - lastTimestamp)) : 0.02;
        initialized = true;
        lastTimestamp = now;
        double delta = target - value;
        double limit = (delta >= 0.0 ? positiveRate : negativeRate) * dt;
        if (Math.abs(delta) <= limit) {
            value = target;
        } else {
            value += Math.copySign(limit, delta);
        }
        return value;
    }

    void reset(double value, double timestamp) {
        this.value = clamp(value);
        lastTimestamp = Double.isFinite(timestamp) ? timestamp : 0.0;
        initialized = true;
    }

    private static double clamp(double value) {
        if (!Double.isFinite(value)) return 0.0;
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
