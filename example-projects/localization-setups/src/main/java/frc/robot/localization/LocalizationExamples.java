package frc.robot.localization;

import ca.frc6390.athena.localization.pipeline.FieldBounds;
import ca.frc6390.athena.localization.pipeline.LocalizationFilters;
import ca.frc6390.athena.localization.pipeline.LocalizationPipeline;
import ca.frc6390.athena.localization.pipeline.Localizations;
import frc.robot.DriveTrain;
import frc.robot.vision.VisionSources;

public final class LocalizationExamples {
    public final LocalizationPipeline odometryOnly;
    public final LocalizationPipeline filteredVision;
    public final LocalizationPipeline weightedFieldPose;
    public final LocalizationPipeline latestCameraPose;

    public LocalizationExamples(DriveTrain driveTrain, VisionSources vision) {
        odometryOnly = Localizations.odometry(driveTrain.odometry)
                .name("odometryOnly");
        filteredVision = Localizations.vision()
                .input(vision.limelightPose, vision.photonPose, vision.heliosPose)
                .filter(LocalizationFilters.latencyLessThan(0.15))
                .filter(LocalizationFilters.tagCountAtLeast(2))
                .filter(FieldBounds.field(16.54, 8.21))
                .name("filteredVision");
        weightedFieldPose = Localizations.weightedAverage()
                .input(odometryOnly, filteredVision)
                .filter(LocalizationFilters.maxPoseJump(odometryOnly, 1.0))
                .publishNetworkTables()
                .name("weightedFieldPose");
        latestCameraPose = Localizations.latestValid()
                .input(vision.limelightPose, vision.photonPose)
                .filter(LocalizationFilters.latencyLessThan(0.25))
                .name("latestCameraPose");
    }
}
