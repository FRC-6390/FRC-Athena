package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.MotorRef;

/**
 * Factories for controller refs.
 */
public final class Control {
    private Control() {
    }

    public static ControlRef position(MotorRef output) {
        return of(ControlMode.POSITION).output(output);
    }

    public static ControlRef velocity(MotorRef output) {
        return of(ControlMode.VELOCITY).output(output);
    }

    public static ControlRef of(ControlMode mode) {
        return new ControlRef(mode, null, null, null, null, null, null, null);
    }
}
