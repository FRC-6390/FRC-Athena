package frc.robot.auto;

import ca.frc6390.athena.vendor.choreo.ChoreoPathAdapter;
import ca.frc6390.athena.vendor.pathplanner.PathPlannerPathProvider;
import choreo.auto.AutoFactory;

/** Shared dependencies passed to every one-file auto example. */
public final class AutoContext {
    public final ExampleDrive drive;
    public final ExampleRobotState state = new ExampleRobotState();
    public final ExampleMechanisms mechanisms = new ExampleMechanisms(state);
    public final PathPlannerPathProvider pathPlanner = new PathPlannerPathProvider();
    public final AutoFactory choreoFactory;
    public final ChoreoPathAdapter choreo;

    public AutoContext(ExampleDrive drive) {
        this.drive = drive;
        choreoFactory = new AutoFactory(
                drive::pose,
                drive::resetPose,
                drive::followChoreoSample,
                true,
                drive);
        choreo = new ChoreoPathAdapter(choreoFactory);
    }
}
