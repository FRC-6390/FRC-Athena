package ca.frc6390.athena.hardware.signal;

/**
 * Mechanism-position input for a control feedback binding.
 */
@FunctionalInterface
public interface PositionSignal extends FeedbackSignal {
    /**
     * Reads the current mechanism position.
     *
     * @return position in the signal's configured mechanism units
     */
    double position();

    /** Returns a position signal using the opposite sign convention. */
    default PositionSignal inverted() {
        return inverted(true);
    }

    /** Selects whether this position signal uses the opposite sign convention. */
    default PositionSignal inverted(boolean inverted) {
        if (!inverted) {
            return this;
        }
        PositionSignal source = this;
        return new PositionSignal() {
            @Override
            public double position() {
                return -source.position();
            }

            @Override
            public java.util.List<?> dependencies() {
                return source.dependencies();
            }
        };
    }
}
