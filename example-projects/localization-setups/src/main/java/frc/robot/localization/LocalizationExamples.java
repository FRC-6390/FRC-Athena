package frc.robot.localization;

import ca.frc6390.athena.localization.pipeline.LocalizationFilters;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.localization.pipeline.VisionFilters;
import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.wpilib.geometry.AllianceTransform;
import frc.robot.DriveTrain;
import frc.robot.vision.VisionSources;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public final class LocalizationExamples implements Mechanism {
    /**
     * Blue-alliance geometry. Blue is the identity; red sees the same layout rotated 180 degrees about the field
     * center. Resolved against the Driver Station on each call.
     */
    public static final AllianceTransform ALLIANCE = AllianceTransform.rotate180(16.54, 8.21);

    /** A pose on the blue side of the field, mapped to the current alliance on every call. */
    public final Supplier<Pose2d> blueStartPose = ALLIANCE.poseSupplier(
            new Pose2d(1.5, 4.0, Rotation2d.kZero));

    public final PoseSignal odometry;
    public final Localization filteredVision;
    public final Localization fusedVision;
    public final Localization weightedFieldPose;
    public final Localization latestCameraPose;
    public final Localization estimatedFieldPose;
    /** One filter stage per camera, so each camera feeds the Kalman node on its own. */
    public final Localization limelightStage;
    public final Localization photonStage;
    public final Localization heliosStage;

    public LocalizationExamples(DriveTrain driveTrain, VisionSources vision) {
        odometry = driveTrain.odometry;
        filteredVision = Localizations.filter()
                .input(vision.limelightPose, vision.photonPose, vision.heliosPose)
                .filter(VisionFilters.finitePoseAndTimestamp())
                .filter(VisionFilters.maxLatencySeconds(0.15))
                .filter(VisionFilters.maxAmbiguity(0.20))
                .filter(VisionFilters.maxAverageTagDistanceMeters(6.0))
                .filter(LocalizationFilters.inside(Rectangle2d.field(16.54, 8.21)))
                .name("filteredVision");
        fusedVision = Localizations.covarianceIntersection()
                .input(filteredVision)
                .groupWithinSeconds(0.03)
                .maxTranslationDisagreementMeters(0.75)
                .maxHeadingDisagreementRadians(Math.toRadians(20.0))
                .name("fusedVision");
        weightedFieldPose = Localizations.weightedAverage()
                .input(odometry, filteredVision)
                .filter(LocalizationFilters.maxPoseJump(odometry, 1.0))
                .publishNetworkTables()
                .name("weightedFieldPose");
        latestCameraPose = Localizations.latestValid()
                .input(vision.limelightPose, vision.photonPose)
                .filter(LocalizationFilters.latencyLessThan(0.25))
                .name("latestCameraPose");
        limelightStage = cameraStage(vision.limelightPose, "limelightStage");
        photonStage = cameraStage(vision.photonPose, "photonStage");
        heliosStage = cameraStage(vision.heliosPose, "heliosStage");
        // Recommended setup: each camera is its own Kalman input, so independent cameras add their information
        // instead of being averaged first. Speed scaling trusts frames less while driving; the spin gate drops frames
        // taken while rotating fast; seeding snaps to a multi-tag pose while disabled, so auto starts from a good pose.
        estimatedFieldPose = Localizations.kalman()
                .input(odometry, limelightStage, photonStage, heliosStage)
                .stateStdDevs(0.08, Math.toRadians(3.0))
                .defaultVisionStdDevs(0.7, Math.toRadians(20.0))
                .visionStdDevSpeedScaling(0.5, 0.25)
                .maxVisionAngularVelocity(3.0)
                .maxNormalizedVisionResidual(9.0)
                .seedFromVisionWhileDisabled(DriverStation::isDisabled)
                .publishNetworkTables()
                .name("estimatedFieldPose");
    }

    /** One camera's filters, named so its status shows on the dashboard as {@code athena/vision/<camera>/...}. */
    private static Localization cameraStage(PoseSignal camera, String name) {
        return Localizations.filter()
                .input(camera)
                .filter(VisionFilters.finitePoseAndTimestamp())
                .filter(VisionFilters.maxLatencySeconds(0.15))
                .filter(VisionFilters.maxAmbiguity(0.20))
                .filter(VisionFilters.maxAverageTagDistanceMeters(6.0))
                .filter(LocalizationFilters.inside(Rectangle2d.field(16.54, 8.21)))
                .name(name);
    }

    public Pose2d pose() {
        return estimatedFieldPose.pose2d();
    }

    /** Zeroes heading relative to the alliance: forward on blue, toward the blue wall on red. */
    public ActionBinding resetHeadingForAlliance() {
        return context -> {
            Pose2d current = pose();
            Rotation2d forward = ALLIANCE.rotation(Rotation2d.kZero);
            estimatedFieldPose.reset(new Pose2d(current.getTranslation(), forward)).apply(context);
        };
    }

    public ActionBinding resetHeading() {
        return context -> {
            Pose2d current = pose();
            estimatedFieldPose.reset(new Pose2d(current.getTranslation(), Rotation2d.kZero)).apply(context);
        };
    }
}
