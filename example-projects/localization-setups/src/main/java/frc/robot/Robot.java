package frc.robot;

import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.localization.LocalizationExamples;
import frc.robot.vision.VisionSources;
import frc.robot.vision.TargetingExamples;

public final class Robot extends AthenaRobot {
    public final DriveTrain driveTrain = new DriveTrain();
    public final VisionSources vision = new VisionSources();
    public final LocalizationExamples localization = new LocalizationExamples(driveTrain, vision);
    public final TargetingExamples targeting = new TargetingExamples(driveTrain, vision);
    public final Controls controls = new Controls(driveTrain, localization, targeting);
}
