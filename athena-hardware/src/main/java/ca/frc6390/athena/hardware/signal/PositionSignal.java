package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.runtime.ActionContext;

/**
 * Mechanism-position input for a control feedback binding.
 */
@FunctionalInterface
public interface PositionSignal extends FeedbackSignal {
    /**
     * Reads the current mechanism position.
     *
     * @param context runtime hardware context
     * @return position in the signal's configured mechanism units
     */
    double position(ActionContext context);
}
