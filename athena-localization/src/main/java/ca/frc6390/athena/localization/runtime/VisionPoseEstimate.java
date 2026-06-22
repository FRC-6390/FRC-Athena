package ca.frc6390.athena.localization.runtime;

import ca.frc6390.athena.localization.spec.VisionWeightSpec;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Timestamped camera-derived robot pose measurement.
 *
 * @param pose estimated robot pose
 * @param timestampSeconds capture timestamp
 * @param standardDeviations measurement standard deviations
 * @param tagCount number of valid tag observations in the source frame
 */
public record VisionPoseEstimate(
        PoseSnapshot pose,
        double timestampSeconds,
        VisionWeightSpec standardDeviations,
        int tagCount) {
}
