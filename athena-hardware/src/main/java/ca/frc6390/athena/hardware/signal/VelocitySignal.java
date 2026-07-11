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
}
