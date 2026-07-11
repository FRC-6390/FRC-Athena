package frc.robot;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.auto.AutoExamples;
import frc.robot.auto.ExampleDrive;

public final class Robot extends AthenaRobot {
    public final ExampleDrive drive = new ExampleDrive();
    public final AutoExamples autos = new AutoExamples(drive);

    @SuppressWarnings("unused")
    public final HookBinding initializeAutos = Events.robotInit().onStart(() -> {
        drive.configurePathPlanner();
        athena().auto(autos.runtime, autos.customMarkers);
    });
}
