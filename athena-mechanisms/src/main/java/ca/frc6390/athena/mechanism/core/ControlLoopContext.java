package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.mechanism.motion.MotionReference;

/**
 * Values a running control loop can read during calculation.
 */
public interface ControlLoopContext {
    Output request();

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

    default double position() {
        return 0.0;
    }

    default double velocity() {
        return 0.0;
    }

    default MotionReference reference() {
        Output output = request();
        if (output instanceof Output.Position position) {
            return MotionReference.stationary(position.position());
        }
        if (output instanceof Output.Velocity velocity) {
            return new MotionReference(position(), velocity.velocity(), 0.0);
        }
        return MotionReference.stationary(position());
    }

    default double dtSeconds() {
        return 0.02;
    }

}
