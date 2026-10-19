package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
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
                VisionSimulationTarget.aprilTag(1, Meters.of(3.0), Meters.of(0.0), Meters.of(1.3), Radians.of(Math.PI)),
                VisionSimulationTarget.aprilTag(2, Meters.of(4.0), Meters.of(1.0), Meters.of(1.3), Radians.of(Math.PI)),
                VisionSimulationTarget.aprilTag(3, Meters.of(5.0), Meters.of(-1.5), Meters.of(1.3), Radians.of(Math.PI))));
    }
}
