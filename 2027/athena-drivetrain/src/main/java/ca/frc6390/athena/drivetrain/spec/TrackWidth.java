package ca.frc6390.athena.drivetrain.spec;

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
}
