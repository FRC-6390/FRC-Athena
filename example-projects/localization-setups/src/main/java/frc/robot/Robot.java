package frc.robot;

import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.localization.LocalizationExamples;
import frc.robot.vision.VisionSources;

public final class Robot extends AthenaRobot {
    public final VisionSources vision = new VisionSources();
    public final LocalizationExamples localization = new LocalizationExamples(vision);

    @Override
    protected void configure() {
        athena()
                .cameras(vision.frontLimelight, vision.rearPhoton, vision.driverHelios)
                .localization(localization.weightedFieldPose)
                .localization(localization.latestCameraPose)
                .localization(localization.kalmanFieldPose)
                .localizationMaxAge(0.5);
    }
}
