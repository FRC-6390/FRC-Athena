package ca.frc6390.athena.drivetrain.swerve;

/**
 * Distance between left and right drivetrain contact patches.
 *
 * @param meters track width in meters
 */
public record TrackWidth(double meters) {
    public TrackWidth {
        if (!Double.isFinite(meters) || meters <= 0.0) {
            throw new IllegalArgumentException("Track width must be positive.");
        }
    }

    /**
     * Creates a track width in meters.
     *
     * @param meters meters
     * @return track width
     */
    public static TrackWidth meters(double meters) {
        return new TrackWidth(meters);
    }

    public static TrackWidth inches(double inches) {
        return meters(inches * 0.0254);
    }

    public static TrackWidth feet(double feet) {
        return inches(feet * 12.0);
    }

    public static TrackWidth centimeters(double centimeters) {
        return meters(centimeters / 100.0);
    }
}
