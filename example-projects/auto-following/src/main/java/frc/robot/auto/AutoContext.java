package frc.robot.auto;

import ca.frc6390.athena.vendor.choreo.ChoreoPathProvider;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Shared dependencies passed to every one-file auto example. */
public final class AutoContext {
    public final ExampleDrive drive;
    public final ExampleRobotState state = new ExampleRobotState();
    public final ExampleMechanisms mechanisms = new ExampleMechanisms(state);
    public final ChoreoPathProvider choreo;

    public AutoContext(ExampleDrive drive) {
        this.drive = drive;
        choreo = ChoreoPathProvider.swerve(
                drive.pathFollower,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
    }
}
