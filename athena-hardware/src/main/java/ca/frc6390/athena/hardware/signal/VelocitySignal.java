package ca.frc6390.athena.hardware.signal;

/**
 * Mechanism-velocity input for a control feedback binding.
 */
@FunctionalInterface
public interface VelocitySignal extends FeedbackSignal {
    /**
     * Reads the current mechanism velocity.
     *
     * @return velocity in the signal's configured mechanism units per second
     */
    double velocity();

    /** Returns a velocity signal using the opposite sign convention. */
    default VelocitySignal inverted() {
        return inverted(true);
    }

    /** Selects whether this velocity signal uses the opposite sign convention. */
    default VelocitySignal inverted(boolean inverted) {
        if (!inverted) {
            return this;
        }
        VelocitySignal source = this;
        return new VelocitySignal() {
            @Override
            public double velocity() {
                return -source.velocity();
            }

            @Override
            public java.util.List<?> dependencies() {
                return source.dependencies();
            }
        };
    }
}
