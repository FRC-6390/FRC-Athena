package frc.robot;

import ca.frc6390.athena.auto.AutoChooser;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.vendor.choreo.ChoreoPathProvider;
import ca.frc6390.athena.vendor.pathplanner.PathPlannerPathProvider;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.auto.AutoContext;
import frc.robot.auto.ChoreoMarkersAuto;
import frc.robot.auto.ChoreoMultiPathSplitAuto;
import frc.robot.auto.CommandInteropExamples;
import frc.robot.auto.ConditionalChoreoAuto;
import frc.robot.auto.ExampleDrive;
import frc.robot.auto.GeneratedPathAuto;
import frc.robot.auto.GeneratedPathProvider;
import frc.robot.auto.PathPlannerAuto;
import frc.robot.auto.PathPlannerSetup;

public final class Robot extends AthenaRobot {
    public final ExampleDrive drive = new ExampleDrive();
    public final AutoContext autoContext = new AutoContext(drive);
    public final PathPlannerSetup pathPlannerSetup = new PathPlannerSetup(drive);
    public final ChoreoPathProvider choreoPaths = autoContext.choreo;
    public final PathPlannerPathProvider pathPlannerPaths = new PathPlannerPathProvider();
    public final GeneratedPathProvider generatedPaths = new GeneratedPathProvider(drive);
    public final CommandInteropExamples commandInterop = new CommandInteropExamples(this);
    public final AutoChooser autos = Autos.chooser()
            .defaultAuto(ChoreoMarkersAuto.create(autoContext))
            .auto(ChoreoMultiPathSplitAuto.create(autoContext))
            .auto(ConditionalChoreoAuto.create(autoContext))
            .auto("PathPlanner Two Piece", PathPlannerAuto.create())
            .auto("Generated Straight Line", GeneratedPathAuto.create());
}
