package ca.frc6390.athena.api.hardware;

/**
 * Controller-plus-physical-motor declaration used by motor devices.
 */
public interface MotorKind extends HardwareKind {
    /** Returns the controller family used to drive this motor, when known. */
    default MotorControllerKind controllerKind() {
        return null;
    }

    /** Returns the physical motor model portion of this declaration. */
    default MotorKind motorKind() {
        return this;
    }

    /** Returns the physical motor's stable model key. */
    default String motorKey() {
        return motorKind().key();
    }

    /** Returns whether this motor is brushed or brushless. */
    default MotorTechnology technology() {
        return MotorTechnology.UNKNOWN;
    }

    /** Returns this physical motor paired with a specific compatible controller. */
    default MotorKind controlledBy(MotorControllerKind controller) {
        return new ControlledMotorKind(controller, motorKind());
    }

    /** Controller-plus-motor descriptor used for non-default pairings. */
    record ControlledMotorKind(MotorControllerKind controllerKind, MotorKind motorKind) implements MotorKind {
        public ControlledMotorKind {
            java.util.Objects.requireNonNull(controllerKind, "controllerKind");
            java.util.Objects.requireNonNull(motorKind, "motorKind");
            if (!controllerKind.supports(motorKind)) {
                throw new IllegalArgumentException(
                        controllerKind.key() + " cannot control physical motor " + motorKind.motorKey() + ".");
            }
        }

        @Override
        public String key() {
            return controllerKind.key() + "/" + motorKind.motorKey();
        }

        @Override
        public String motorKey() {
            return motorKind.motorKey();
        }

        @Override
        public MotorTechnology technology() {
            return motorKind.technology();
        }
    }
}
