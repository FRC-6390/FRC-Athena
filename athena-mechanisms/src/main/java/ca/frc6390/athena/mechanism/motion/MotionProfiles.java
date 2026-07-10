package ca.frc6390.athena.mechanism.motion;

/**
 * Factories for scalar motion profiles.
 */
public final class MotionProfiles {
    private MotionProfiles() {
    }

    public static MotionProfile trapezoid(double maxVelocity, double maxAcceleration) {
        return new MotionProfile(maxVelocity, maxAcceleration);
    }
}
