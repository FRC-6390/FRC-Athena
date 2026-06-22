package ca.frc6390.athena.drivetrain.config;

/**
 * Entry points for drivetrain declarations.
 */
public final class Drivetrains {
    private Drivetrains() {
    }

    /**
     * Creates a differential drivetrain declaration.
     *
     * @param name drivetrain name
     * @return drivetrain config
     */
    public static DifferentialDrivetrainConfig differential(String name) {
        return new DifferentialDrivetrainConfig(name);
    }

    /**
     * Creates a swerve drivetrain declaration.
     *
     * @param name drivetrain name
     * @return drivetrain config
     */
    public static SwerveDrivetrainConfig swerve(String name) {
        return new SwerveDrivetrainConfig(name);
    }
}
