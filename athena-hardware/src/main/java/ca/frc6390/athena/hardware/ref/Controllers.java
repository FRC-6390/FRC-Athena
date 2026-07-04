package ca.frc6390.athena.hardware.ref;

/**
 * Factories for controllers.
 */
public final class Controllers {
    private Controllers() {
    }

    /**
     * Creates an Xbox-style controller declaration.
     *
     * @param port driver station port
     * @return controller ref
     */
    public static ControllerRef xbox(int port) {
        return ControllerRef.xbox(port);
    }
}
