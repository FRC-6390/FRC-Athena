package frc.robot;

import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.localization.LocalizationExamples;
import frc.robot.vision.VisionSources;

public final class Robot extends AthenaRobot {
    public final DriveTrain driveTrain = new DriveTrain();
    public final VisionSources vision = new VisionSources();
    public final LocalizationExamples localization = new LocalizationExamples(driveTrain, vision);

    @Override
    protected void configure() {
        register(driveTrain);
        athena()
                .cameras(vision.frontLimelight, vision.rearPhoton, vision.driverHelios)
                .localization(localization.estimatedFieldPose)
                .localization(localization.latestCameraPose)
                .localizationMaxAge(0.5);
    }
}
