package ca.frc6390.athena.mechanism.spec;

/**
 * Simple feedforward gain declaration.
 *
 * @param staticGain static friction/feedforward gain
 * @param velocityGain velocity gain
 * @param gravityGain gravity compensation gain
 */
public record FeedforwardSpec(double staticGain, double velocityGain, double gravityGain) {
    /**
     * Returns whether all gains are finite.
     *
     * @return true if valid
     */
    public boolean isFinite() {
        return Double.isFinite(staticGain) && Double.isFinite(velocityGain) && Double.isFinite(gravityGain);
    }
}
