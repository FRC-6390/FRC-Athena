package ca.frc6390.athena.mechanism.motion;

/**
 * Position, velocity, and acceleration reference produced by a motion profile.
 */
public record MotionReference(double position, double velocity, double acceleration) {
    public MotionReference {
        if (!Double.isFinite(position) || !Double.isFinite(velocity) || !Double.isFinite(acceleration)) {
            throw new IllegalArgumentException("Motion reference values must be finite.");
        }
    }

    public static MotionReference stationary(double position) {
        return new MotionReference(position, 0.0, 0.0);
    }
}
