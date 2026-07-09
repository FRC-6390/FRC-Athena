package ca.frc6390.athena.drivetrain.swerve;

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

    public static WheelBase inches(double inches) {
        return meters(inches * 0.0254);
    }

    public static WheelBase feet(double feet) {
        return inches(feet * 12.0);
    }

    public static WheelBase centimeters(double centimeters) {
        return meters(centimeters / 100.0);
    }
}
