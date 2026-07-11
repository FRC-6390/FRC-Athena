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
}
