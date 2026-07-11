package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.wpilib.commands.WpilibCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;

/** Level 5: generate a short path from live robot state when the auto starts. */
public final class DynamicPathPlannerAuto {
    private static final PathConstraints CONSTRAINTS =
            new PathConstraints(2.5, 2.0, Math.PI, Math.PI * 2.0);

    private DynamicPathPlannerAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        return Autos.routine("PP 5 - Dynamic on-the-fly", () -> {
            Command command = Commands.defer(
                    () -> AutoBuilder.followPath(buildPath(context)),
                    Set.of(context.drive));
            command.setName("pp-dynamic-short-path");
            return WpilibCommands.wrap(command);
        });
    }

    private static PathPlannerPath buildPath(AutoContext context) {
        Pose2d start = context.drive.pose();
        Pose2d goal = context.state.visionTargetVisible()
                ? new Pose2d(3.25, 5.55, Rotation2d.fromDegrees(180.0))
                : new Pose2d(2.60, 5.10, Rotation2d.fromDegrees(180.0));

        Rotation2d travelHeading = goal.getTranslation()
                .minus(start.getTranslation())
                .getAngle();
        Pose2d startTangent = new Pose2d(start.getTranslation(), travelHeading);
        Pose2d endTangent = new Pose2d(goal.getTranslation(), travelHeading);

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(startTangent, endTangent),
                CONSTRAINTS,
                new IdealStartingState(0.0, start.getRotation()),
                new GoalEndState(0.0, goal.getRotation()));
        // The poses are already absolute field coordinates from localization.
        path.preventFlipping = true;
        return path;
    }
}
