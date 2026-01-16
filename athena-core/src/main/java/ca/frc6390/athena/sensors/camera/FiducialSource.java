package ca.frc6390.athena.sensors.camera;

import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Camera interface for providers that can detect fiducials (ex: AprilTags).
 */
public interface FiducialSource {
    OptionalInt getLatestTagId();

    List<VisionCamera.TargetMeasurement> getTargetMeasurements();

    Optional<VisionCamera.TargetObservation> getLatestObservation(
            VisionCamera.CoordinateSpace space, Pose2d robotPose, Pose2d tagPose);

    Optional<Pose2d> estimateFieldPoseFromTag(int tagId);

    Optional<Pose2d> estimateFieldPoseFromTag(int tagId, Translation2d cameraOffsetMeters);
}
