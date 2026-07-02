package ca.frc6390.athena.mechanism.ref;

import ca.frc6390.athena.mechanism.spec.FeedforwardSpec;

/**
 * Reusable feedforward declaration for robot constants.
 */
public record FeedforwardRef(double staticGain, double velocityGain, double gravityGain) {
    public static FeedforwardRef of(double staticGain, double velocityGain, double gravityGain) {
        return new FeedforwardRef(staticGain, velocityGain, gravityGain);
    }

    public static FeedforwardRef simple(double staticGain, double velocityGain, double gravityGain) {
        return of(staticGain, velocityGain, gravityGain);
    }

    public FeedforwardSpec toSpec() {
        return new FeedforwardSpec(staticGain, velocityGain, gravityGain);
    }
}
