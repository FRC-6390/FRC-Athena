package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.ActionContext;

/**
 * Values a running control loop can read during calculation.
 */
public interface ControlLoopContext {
    /**
     * Returns the requested output that activated this loop.
     *
     * @return requested output
     */
    Output request();

    /**
     * Returns the scalar target from the requested output.
     *
     * @return target
     */
    default double target() {
        Output output = request();
        if (output instanceof Output.Percent percent) {
            return percent.percent();
        }
        if (output instanceof Output.Voltage voltage) {
            return voltage.volts();
        }
        if (output instanceof Output.Position position) {
            return position.position();
        }
        if (output instanceof Output.Velocity velocity) {
            return velocity.velocity();
        }
        return 0.0;
    }

    /**
     * Returns current measured position.
     *
     * @return position
     */
    default double position() {
        return 0.0;
    }

    /**
     * Returns current measured velocity.
     *
     * @return velocity
     */
    default double velocity() {
        return 0.0;
    }

    /**
     * Returns loop period.
     *
     * @return dt in seconds
     */
    default double dtSeconds() {
        return 0.02;
    }

    /**
     * Returns constraints attached to the active control.
     *
     * @return constraints
     */
    default ConstraintRef constraints() {
        return ConstraintRef.none();
    }

    /**
     * Returns runtime ref access.
     *
     * @return action context
     */
    default ActionContext refs() {
        return ActionContext.empty();
    }
}
