package ca.frc6390.athena.localization.spec;

/**
 * Slip detection thresholds used by drivetrain localization adapters.
 *
 * @param enabled true when slip detection is active
 * @param lateralVelocityMetersPerSecond lateral velocity threshold
 * @param angularVelocityRadiansPerSecond angular velocity threshold
 */
public record SlipDetectionSpec(
        boolean enabled,
        double lateralVelocityMetersPerSecond,
        double angularVelocityRadiansPerSecond) {
    /**
     * Creates enabled slip detection thresholds.
     *
     * @param lateralVelocityMetersPerSecond lateral velocity threshold
     * @param angularVelocityRadiansPerSecond angular velocity threshold
     * @return slip detection spec
     */
    public static SlipDetectionSpec enabled(
            double lateralVelocityMetersPerSecond,
            double angularVelocityRadiansPerSecond) {
        return new SlipDetectionSpec(true, lateralVelocityMetersPerSecond, angularVelocityRadiansPerSecond);
    }

    /**
     * Creates disabled slip detection settings.
     *
     * @return disabled slip detection
     */
    public static SlipDetectionSpec disabled() {
        return new SlipDetectionSpec(false, 0.0, 0.0);
    }

    /**
     * Returns true when thresholds are usable.
     *
     * @return true if valid
     */
    public boolean isValid() {
        return Double.isFinite(lateralVelocityMetersPerSecond)
                && Double.isFinite(angularVelocityRadiansPerSecond)
                && lateralVelocityMetersPerSecond >= 0.0
                && angularVelocityRadiansPerSecond >= 0.0;
    }
}
