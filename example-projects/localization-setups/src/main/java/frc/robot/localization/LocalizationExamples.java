package frc.robot.localization;

import ca.frc6390.athena.localization.pipeline.FieldBounds;
import ca.frc6390.athena.localization.pipeline.LocalizationFilters;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.localization.pipeline.VisionFilters;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import frc.robot.DriveTrain;
import frc.robot.vision.VisionSources;

public final class LocalizationExamples {
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
                .filter(FieldBounds.field(16.54, 8.21))
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
                .name("estimatedFieldPose");
    }
}
