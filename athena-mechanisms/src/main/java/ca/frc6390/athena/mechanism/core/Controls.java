package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Factories for control bindings.
 */
public final class Controls {
    private Controls() {
    }

    public static ControlBinding position(MotorDevice output) {
        return of(ControlMode.POSITION).output(output);
    }

    /** Creates one position control that commands every supplied motor. */
    public static ControlBinding position(MotorDevice output, MotorDevice... additionalOutputs) {
        return outputs(ControlMode.POSITION, output, additionalOutputs);
    }

    public static ControlBinding velocity(MotorDevice output) {
        return of(ControlMode.VELOCITY).output(output);
    }

    /** Creates one velocity control that commands every supplied motor. */
    public static ControlBinding velocity(MotorDevice output, MotorDevice... additionalOutputs) {
        return outputs(ControlMode.VELOCITY, output, additionalOutputs);
    }

    public static ControlBinding of(ControlMode mode) {
        return new ControlBinding(mode, null, 0, null, null, null, null);
    }

    private static ControlBinding outputs(
            ControlMode mode,
            MotorDevice output,
            MotorDevice... additionalOutputs) {
        return of(mode).output(output).followers(additionalOutputs);
    }
}
