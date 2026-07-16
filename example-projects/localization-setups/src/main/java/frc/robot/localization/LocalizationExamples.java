package frc.robot.localization;

import ca.frc6390.athena.localization.pipeline.LocalizationFilters;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.localization.pipeline.VisionFilters;
import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import frc.robot.DriveTrain;
import frc.robot.vision.VisionSources;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class LocalizationExamples implements Mechanism {
    public final PoseSignal odometry;
    public final Localization filteredVision;
    public final Localization fusedVision;
    public final Localization weightedFieldPose;
    public final Localization latestCameraPose;
    public final Localization estimatedFieldPose;

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
        estimatedFieldPose = Localizations.kalman()
                .input(odometry, fusedVision)
                .stateStdDevs(0.08, Math.toRadians(3.0))
                .defaultVisionStdDevs(0.7, Math.toRadians(20.0))
                .maxNormalizedVisionResidual(9.0)
                .publishNetworkTables()
                .name("estimatedFieldPose");
    }

    public Pose2d pose() {
        return estimatedFieldPose.pose2d();
    }

    public ActionBinding resetHeading() {
        return context -> {
            Pose2d current = pose();
            estimatedFieldPose.reset(new Pose2d(current.getTranslation(), Rotation2d.kZero)).apply(context);
        };
    }
}
