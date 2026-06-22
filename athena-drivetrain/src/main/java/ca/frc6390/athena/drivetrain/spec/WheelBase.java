package ca.frc6390.athena.drivetrain.spec;

/**
 * Distance between front and back drivetrain contact patches.
 *
 * @param meters wheelbase in meters
 */
public record WheelBase(double meters) {
    public WheelBase {
        if (!Double.isFinite(meters) || meters <= 0.0) {
            throw new IllegalArgumentException("Wheelbase must be positive.");
        }
    }

    /**
     * Creates a wheelbase in meters.
     *
     * @param meters meters
     * @return wheelbase
     */
    public static WheelBase meters(double meters) {
        return new WheelBase(meters);
    }
}
