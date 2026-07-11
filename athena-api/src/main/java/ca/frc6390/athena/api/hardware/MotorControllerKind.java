package ca.frc6390.athena.api.hardware;

/**
 * Stable Athena key for a motor-controller family.
 */
public interface MotorControllerKind extends HardwareKind {
    /** Returns whether this controller supports the supplied physical motor declaration. */
    default boolean supports(MotorKind motor) {
        return true;
    }
}
