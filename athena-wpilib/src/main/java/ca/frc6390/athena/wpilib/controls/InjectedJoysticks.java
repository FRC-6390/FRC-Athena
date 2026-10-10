package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import ca.frc6390.athena.robot.RuntimeInputs;

/**
 * Lookup of recorded joystick state while a replay injects input. Controller adapters ask here first and fall back
 * to the live WPILib driver station when nothing is injected.
 */
final class InjectedJoysticks {
    private InjectedJoysticks() {
    }

    /** Returns the injected state of a port, or null when the live driver station should be read. */
    static RuntimeInputSnapshot.Joystick at(int port) {
        return RuntimeInputs.injected().map(snapshot -> snapshot.joystick(port)).orElse(null);
    }
}
