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

    public static ControlBinding velocity(MotorDevice output) {
        return of(ControlMode.VELOCITY).output(output);
    }

    public static ControlBinding of(ControlMode mode) {
        return new ControlBinding(mode, null, 0, null, null, null, null);
    }
}
