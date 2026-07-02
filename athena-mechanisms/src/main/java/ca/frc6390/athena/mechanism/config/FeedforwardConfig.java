package ca.frc6390.athena.mechanism.config;

import ca.frc6390.athena.mechanism.spec.FeedforwardSpec;
import ca.frc6390.athena.mechanism.ref.FeedforwardRef;

/**
 * Student-facing feedforward gain builder.
 */
public final class FeedforwardConfig {
    private double staticGain;
    private double velocityGain;
    private double gravityGain;

    /**
     * Sets static gain.
     *
     * @param gain gain
     * @return this config
     */
    public FeedforwardConfig staticGain(double gain) {
        staticGain = gain;
        return this;
    }

    /**
     * Sets velocity gain.
     *
     * @param gain gain
     * @return this config
     */
    public FeedforwardConfig velocity(double gain) {
        velocityGain = gain;
        return this;
    }

    /**
     * Sets gravity compensation gain.
     *
     * @param gain gain
     * @return this config
     */
    public FeedforwardConfig gravity(double gain) {
        gravityGain = gain;
        return this;
    }

    /**
     * Applies a reusable feedforward reference.
     *
     * @param feedforward feedforward reference
     * @return this config
     */
    public FeedforwardConfig apply(FeedforwardRef feedforward) {
        staticGain = feedforward.staticGain();
        velocityGain = feedforward.velocityGain();
        gravityGain = feedforward.gravityGain();
        return this;
    }

    /**
     * Lowers to immutable feedforward gains.
     *
     * @return feedforward spec
     */
    public FeedforwardSpec toSpec() {
        return new FeedforwardSpec(staticGain, velocityGain, gravityGain);
    }
}
