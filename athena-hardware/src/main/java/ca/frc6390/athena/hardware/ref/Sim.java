package ca.frc6390.athena.hardware.ref;

import java.util.OptionalDouble;

/**
 * Factories for hardware-linked simulation profiles.
 */
public final class Sim {
    private Sim() {
    }

    /**
     * Creates a generic motor simulation profile.
     *
     * @return sim profile
     */
    public static SimRef motor() {
        return SimRef.of(SimProfile.Kind.MOTOR);
    }

    /**
     * Creates a generic motor simulation model bound to motors.
     *
     * @param motors simulated motors
     * @return sim ref
     */
    public static SimRef motor(MotorRef... motors) {
        return motor().motors(motors);
    }

    /**
     * Creates an arm simulation profile.
     *
     * @return sim profile
     */
    public static SimRef arm() {
        return new SimRef(SimProfile.Kind.ARM, OptionalDouble.empty(), OptionalDouble.empty(), null, true, null, null, null);
    }

    /**
     * Creates an arm simulation model bound to motors.
     *
     * @param motors simulated motors
     * @return sim ref
     */
    public static SimRef arm(MotorRef... motors) {
        return arm().motors(motors);
    }

    /**
     * Creates a flywheel simulation profile.
     *
     * @return sim profile
     */
    public static SimRef flywheel() {
        return SimRef.of(SimProfile.Kind.FLYWHEEL);
    }

    /**
     * Creates a flywheel simulation model bound to motors.
     *
     * @param motors simulated motors
     * @return sim ref
     */
    public static SimRef flywheel(MotorRef... motors) {
        return flywheel().motors(motors);
    }
}
