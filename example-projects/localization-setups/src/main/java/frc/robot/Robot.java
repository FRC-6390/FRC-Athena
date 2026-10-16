package frc.robot;

import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.localization.LocalizationExamples;
import frc.robot.vision.VisionSources;
import frc.robot.vision.TargetingExamples;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import ca.frc6390.athena.vision.runtime.VisionSimulationTarget;

public final class Robot extends AthenaRobot {
    public final DriveTrain driveTrain = new DriveTrain();
    public final VisionSources vision = new VisionSources();
    public final LocalizationExamples localization = new LocalizationExamples(driveTrain, vision);
    public final TargetingExamples targeting = new TargetingExamples(driveTrain, vision);
    public final Controls controls = new Controls(driveTrain, localization, targeting);

    public Robot() {
        // Simulation only: the tags that simulated PhotonVision/Limelight/Helios cameras see. The sim robot starts at
        // the origin facing +X, so the first two tags are straight ahead and the third is off to the side.
        simulationVisionField(VisionSimulationField.of(
                VisionSimulationTarget.aprilTag(1, 3.0, 0.0, 1.3, Math.PI),
                VisionSimulationTarget.aprilTag(2, 4.0, 1.0, 1.3, Math.PI),
                VisionSimulationTarget.aprilTag(3, 5.0, -1.5, 1.3, Math.PI)));
    }
}
