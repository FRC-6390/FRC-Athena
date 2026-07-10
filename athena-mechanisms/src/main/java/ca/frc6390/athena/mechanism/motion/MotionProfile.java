package ca.frc6390.athena.mechanism.motion;

/**
 * Velocity and acceleration limits for a scalar control reference.
 *
 * @param maxVelocity maximum absolute velocity in feedback units per second
 * @param maxAcceleration maximum absolute acceleration in feedback units per second squared
 */
public record MotionProfile(double maxVelocity, double maxAcceleration) {
    public MotionProfile {
        if (!Double.isFinite(maxVelocity) || maxVelocity <= 0.0) {
            throw new IllegalArgumentException("Maximum velocity must be positive.");
        }
        if (!Double.isFinite(maxAcceleration) || maxAcceleration <= 0.0) {
            throw new IllegalArgumentException("Maximum acceleration must be positive.");
        }
    }
}
