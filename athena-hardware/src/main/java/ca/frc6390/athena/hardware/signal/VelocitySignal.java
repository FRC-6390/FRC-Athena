package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.runtime.ActionContext;

/**
 * Mechanism-velocity input for a control feedback binding.
 */
@FunctionalInterface
public interface VelocitySignal extends FeedbackSignal {
    /**
     * Reads the current mechanism velocity.
     *
     * @param context runtime hardware context
     * @return velocity in the signal's configured mechanism units per second
     */
    double velocity(ActionContext context);
}
