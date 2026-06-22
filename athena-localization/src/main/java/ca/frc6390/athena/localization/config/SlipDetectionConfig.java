package ca.frc6390.athena.localization.config;

import ca.frc6390.athena.localization.spec.SlipDetectionSpec;

/**
 * Mutable slip detection declaration.
 */
public final class SlipDetectionConfig {
    private boolean enabled;
    private double lateralVelocityMetersPerSecond;
    private double angularVelocityRadiansPerSecond;

    private SlipDetectionConfig(SlipDetectionSpec spec) {
        enabled = spec.enabled();
        lateralVelocityMetersPerSecond = spec.lateralVelocityMetersPerSecond();
        angularVelocityRadiansPerSecond = spec.angularVelocityRadiansPerSecond();
    }

    static SlipDetectionConfig from(SlipDetectionSpec spec) {
        return new SlipDetectionConfig(spec);
    }

    /**
     * Enables slip detection with thresholds.
     *
     * @param lateralVelocityMetersPerSecond lateral velocity threshold
     * @param angularVelocityRadiansPerSecond angular velocity threshold
     * @return this config
     */
    public SlipDetectionConfig enabled(
            double lateralVelocityMetersPerSecond,
            double angularVelocityRadiansPerSecond) {
        enabled = true;
        this.lateralVelocityMetersPerSecond = lateralVelocityMetersPerSecond;
        this.angularVelocityRadiansPerSecond = angularVelocityRadiansPerSecond;
        return this;
    }

    /**
     * Disables slip detection for drivetrains without reliable slip signals.
     *
     * @return this config
     */
    public SlipDetectionConfig disabled() {
        enabled = false;
        return this;
    }

    SlipDetectionSpec toSpec() {
        return new SlipDetectionSpec(
                enabled,
                lateralVelocityMetersPerSecond,
                angularVelocityRadiansPerSecond);
    }
}
