package ca.frc6390.athena.wpilib.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
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

    /** Creates a PlayStation controller using WPILib's standard PS4 HID layout. */
    public static PlayStationGamepad playstation(int port) {
        return new PlayStationGamepad(new PS4Controller(port));
    }

    /** Alias for {@link #playstation(int)}. */
    public static PlayStationGamepad ps4(int port) {
        return playstation(port);
    }

    /** Creates an unmapped HID controller addressed by raw axis and button IDs. */
    public static GenericController generic(int port) {
        return new GenericController(new GenericHID(port));
    }
}
