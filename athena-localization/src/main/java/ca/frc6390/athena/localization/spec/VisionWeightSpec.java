package ca.frc6390.athena.localization.spec;

/**
 * Vision measurement standard deviations for pose estimation.
 *
 * @param xStdDevMeters x standard deviation
 * @param yStdDevMeters y standard deviation
 * @param headingStdDevRadians heading standard deviation
 * @param multiTagScale multiplier for multi-tag measurements
 */
public record VisionWeightSpec(
        double xStdDevMeters,
        double yStdDevMeters,
        double headingStdDevRadians,
        double multiTagScale) {
    /**
     * Creates default vision weighting.
     *
     * @return default weighting
     */
    public static VisionWeightSpec defaults() {
        return new VisionWeightSpec(0.9, 0.9, 0.7, 0.55);
    }

    /**
     * Returns weighting for a measurement with the given tag count.
     *
     * @param tagCount number of AprilTags in the measurement
     * @return scaled weighting
     */
    public VisionWeightSpec forTagCount(int tagCount) {
        if (tagCount <= 1) {
            return this;
        }
        return new VisionWeightSpec(
                xStdDevMeters * multiTagScale,
                yStdDevMeters * multiTagScale,
                headingStdDevRadians * multiTagScale,
                multiTagScale);
    }

    /**
     * Returns true when weighting values are usable.
     *
     * @return true if valid
     */
    public boolean isValid() {
        return Double.isFinite(xStdDevMeters)
                && Double.isFinite(yStdDevMeters)
                && Double.isFinite(headingStdDevRadians)
                && Double.isFinite(multiTagScale)
                && xStdDevMeters > 0.0
                && yStdDevMeters > 0.0
                && headingStdDevRadians > 0.0
                && multiTagScale > 0.0;
    }
}
