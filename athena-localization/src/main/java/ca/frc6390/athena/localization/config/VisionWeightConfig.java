package ca.frc6390.athena.localization.config;

import ca.frc6390.athena.localization.spec.VisionWeightSpec;

/**
 * Mutable vision measurement weighting declaration.
 */
public final class VisionWeightConfig {
    private double xStdDevMeters = 0.9;
    private double yStdDevMeters = 0.9;
    private double headingStdDevRadians = 0.7;
    private double multiTagScale = 0.55;

    /**
     * Sets single-target standard deviations.
     *
     * @param xStdDevMeters x standard deviation
     * @param yStdDevMeters y standard deviation
     * @param headingStdDevRadians heading standard deviation
     * @return this config
     */
    public VisionWeightConfig standardDeviations(
            double xStdDevMeters,
            double yStdDevMeters,
            double headingStdDevRadians) {
        this.xStdDevMeters = xStdDevMeters;
        this.yStdDevMeters = yStdDevMeters;
        this.headingStdDevRadians = headingStdDevRadians;
        return this;
    }

    /**
     * Sets the multiplier applied to deviations for multi-tag measurements.
     *
     * @param multiTagScale multi-tag scale
     * @return this config
     */
    public VisionWeightConfig multiTagScale(double multiTagScale) {
        this.multiTagScale = multiTagScale;
        return this;
    }

    VisionWeightSpec toSpec() {
        return new VisionWeightSpec(xStdDevMeters, yStdDevMeters, headingStdDevRadians, multiTagScale);
    }
}
