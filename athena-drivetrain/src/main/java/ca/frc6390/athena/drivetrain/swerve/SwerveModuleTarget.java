package ca.frc6390.athena.drivetrain.swerve;

/**
 * Runtime target for a single swerve module.
 *
 * @param speedMetersPerSecond requested wheel speed
 * @param angleRotations requested steering angle, in mechanism rotations
 */
public record SwerveModuleTarget(double speedMetersPerSecond, double angleRotations) {
    public SwerveModuleTarget {
        requireFinite(speedMetersPerSecond, "speedMetersPerSecond");
        requireFinite(angleRotations, "angleRotations");
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }
}
