package ca.frc6390.athena.wpilib.controls;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Controller declaration factories.
 */
public final class Controllers {
    private Controllers() {
    }

    public static Gamepad xbox(int port) {
        return new Gamepad(new XboxController(port));
    }
}
