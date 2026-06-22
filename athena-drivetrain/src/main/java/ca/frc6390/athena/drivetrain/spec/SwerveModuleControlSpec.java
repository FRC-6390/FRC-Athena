package ca.frc6390.athena.drivetrain.spec;

/**
 * Normalized swerve module control gains.
 *
 * @param steerP steer proportional gain
 * @param steerI steer integral gain
 * @param steerD steer derivative gain
 * @param driveKs drive static feedforward gain
 * @param driveKv drive velocity feedforward gain
 * @param driveKa drive acceleration feedforward gain
 */
public record SwerveModuleControlSpec(
        double steerP,
        double steerI,
        double steerD,
        double driveKs,
        double driveKv,
        double driveKa) {
    /**
     * Returns whether all gains are finite.
     *
     * @return true if every gain is finite
     */
    public boolean isFinite() {
        return Double.isFinite(steerP)
                && Double.isFinite(steerI)
                && Double.isFinite(steerD)
                && Double.isFinite(driveKs)
                && Double.isFinite(driveKv)
                && Double.isFinite(driveKa);
    }
}
