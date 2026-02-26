package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.localization.PoseBoundingBox2d;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Example localization configuration patterns.
 */
public final class LocalizationExamples {
    private LocalizationExamples() {}

    public static RobotLocalizationConfig createCompetitionConfig() {
        return RobotLocalizationConfig.create()
                .estimation(e -> e
                        .vision(0.6, 0.6, 6.0)
                        .visionMultitag(0.35, 0.35, 3.0)
                        .visionEnabled(true))
                .backendConfig(b -> b
                        .slipYawRateThreshold(0.45)
                        .slipAccelThreshold(1.2)
                        .poseJumpMeters(0.8))
                .poses(p -> p
                        .pose("field", pose -> pose
                                .odometry()
                                .publishToNetworkTables(true)
                                .backend(backend -> backend
                                        .visionStrategy(RobotLocalizationConfig.BackendConfig.VisionStrategy.DISABLED)
                                        .imuStrategy(RobotLocalizationConfig.BackendConfig.ImuStrategy.VIRTUAL_AXES)
                                        .slip(slip -> slip.enabled(false))))
                        .boundingBox("loading", new Translation2d(1.0, 1.0), new Translation2d(2.0, 3.0))
                        .boundingBox("source", new PoseBoundingBox2d(5.0, 7.0, 0.5, 2.0))
                        .autoPoseName("loading"));
    }
}
