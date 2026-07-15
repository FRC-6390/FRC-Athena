package ca.frc6390.athena.mechanism.core;

/** Cached component contributions from the latest software control-loop calculation. */
public record ControlLoopTrace(
        double proportionalVolts,
        double integralVolts,
        double derivativeVolts,
        double staticFeedforwardVolts,
        double velocityFeedforwardVolts,
        double accelerationFeedforwardVolts,
        double gravityFeedforwardVolts) {
    public static final ControlLoopTrace ZERO = new ControlLoopTrace(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /** Adds independently composed loop contributions. */
    public ControlLoopTrace plus(ControlLoopTrace other) {
        if (other == null) {
            return this;
        }
        return new ControlLoopTrace(
                proportionalVolts + other.proportionalVolts,
                integralVolts + other.integralVolts,
                derivativeVolts + other.derivativeVolts,
                staticFeedforwardVolts + other.staticFeedforwardVolts,
                velocityFeedforwardVolts + other.velocityFeedforwardVolts,
                accelerationFeedforwardVolts + other.accelerationFeedforwardVolts,
                gravityFeedforwardVolts + other.gravityFeedforwardVolts);
    }
}
