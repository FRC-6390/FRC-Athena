package ca.frc6390.athena.localization.runtime;

import ca.frc6390.athena.localization.spec.LocalizationSpec;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.vision.spec.CameraMountPose;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Converts generic camera target frames into localization pose measurements.
 */
public final class VisionPoseEstimator {
    private final LocalizationSpec localization;
    private final CameraMountPose cameraMount;
    private final Map<Integer, PoseSnapshot> fieldTags;
    private final DoubleSupplier headingRadians;
    private final DoubleSupplier timestampSeconds;

    /**
     * Creates a camera pose estimator.
     *
     * @param localization localization spec with vision weights
     * @param cameraMount robot-relative camera mount
     * @param fieldTags AprilTag field poses keyed by tag id
     * @param headingRadians current robot heading
     * @param timestampSeconds current capture timestamp
     */
    public VisionPoseEstimator(
            LocalizationSpec localization,
            CameraMountPose cameraMount,
            Map<Integer, PoseSnapshot> fieldTags,
            DoubleSupplier headingRadians,
            DoubleSupplier timestampSeconds) {
        this.localization = Objects.requireNonNull(localization, "localization");
        this.cameraMount = cameraMount == null ? CameraMountPose.identity() : cameraMount;
        this.fieldTags = Map.copyOf(Objects.requireNonNull(fieldTags, "fieldTags"));
        this.headingRadians = Objects.requireNonNull(headingRadians, "headingRadians");
        this.timestampSeconds = Objects.requireNonNull(timestampSeconds, "timestampSeconds");
    }

    /**
     * Estimates a robot pose from the best valid tag in a frame.
     *
     * @param frame vision frame
     * @return pose estimate when a known valid tag exists
     */
    public Optional<VisionPoseEstimate> estimate(VisionFrame frame) {
        if (frame == null) {
            return Optional.empty();
        }
        Optional<VisionObservation> target = frame.bestTarget()
                .filter(observation -> observation.tagId() >= 0)
                .filter(observation -> fieldTags.containsKey(observation.tagId()));
        if (target.isEmpty()) {
            return Optional.empty();
        }

        int tagCount = (int) frame.validObservations().stream()
                .filter(observation -> observation.tagId() >= 0)
                .filter(observation -> fieldTags.containsKey(observation.tagId()))
                .count();
        PoseSnapshot tagPose = fieldTags.get(target.get().tagId());
        PoseSnapshot robotPose = estimateRobotPose(tagPose, target.get());
        if (!localization.fieldBounds().contains(robotPose)) {
            return Optional.empty();
        }
        return Optional.of(new VisionPoseEstimate(
                robotPose,
                timestampSeconds.getAsDouble(),
                localization.visionWeight().forTagCount(tagCount),
                tagCount));
    }

    private PoseSnapshot estimateRobotPose(PoseSnapshot tagPose, VisionObservation observation) {
        double robotHeading = headingRadians.getAsDouble();
        double cameraHeading = robotHeading + Math.toRadians(cameraMount.yawDegrees());
        double targetBearing = cameraHeading + Math.toRadians(observation.yawDegrees());
        double cameraX = tagPose.xMeters() - observation.distanceMeters() * Math.cos(targetBearing);
        double cameraY = tagPose.yMeters() - observation.distanceMeters() * Math.sin(targetBearing);

        double mountX = cameraMount.xMeters() * Math.cos(robotHeading)
                - cameraMount.yMeters() * Math.sin(robotHeading);
        double mountY = cameraMount.xMeters() * Math.sin(robotHeading)
                + cameraMount.yMeters() * Math.cos(robotHeading);
        return new PoseSnapshot(cameraX - mountX, cameraY - mountY, robotHeading);
    }
}
