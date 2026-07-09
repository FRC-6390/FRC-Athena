package ca.frc6390.athena.hardware.sim;

import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.OptionalDouble;

/**
 * Factories for hardware-linked simulation profiles.
 */
public final class SimModels {
    private SimModels() {
    }

    /**
     * Creates a generic motor simulation profile.
     *
     * @return sim profile
     */
    public static SimModel motor() {
        return SimModel.of(SimProfile.Kind.MOTOR);
    }

    /**
     * Creates a generic motor simulation model bound to motors.
     *
     * @param motors simulated motors
     * @return sim ref
     */
    public static SimModel motor(MotorDevice... motors) {
        return motor().motors(motors);
    }

    /**
     * Creates an arm simulation profile.
     *
     * @return sim profile
     */
    public static SimModel arm() {
        return new SimModel(SimProfile.Kind.ARM, OptionalDouble.empty(), OptionalDouble.empty(), null, true, null, null, null);
    }

    /**
     * Creates an arm simulation model bound to motors.
     *
     * @param motors simulated motors
     * @return sim ref
     */
    public static SimModel arm(MotorDevice... motors) {
        return arm().motors(motors);
    }

    /**
     * Creates a flywheel simulation profile.
     *
     * @return sim profile
     */
    public static SimModel flywheel() {
        return SimModel.of(SimProfile.Kind.FLYWHEEL);
    }

    /**
     * Creates a flywheel simulation model bound to motors.
     *
     * @param motors simulated motors
     * @return sim ref
     */
    public static SimModel flywheel(MotorDevice... motors) {
        return flywheel().motors(motors);
    }
}
