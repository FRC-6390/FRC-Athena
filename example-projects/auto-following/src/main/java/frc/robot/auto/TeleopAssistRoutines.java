package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Autonomous-style routines intended to be scheduled from driver controls in teleop. */
public final class TeleopAssistRoutines {
    private static final PathConstraints ASSIST_CONSTRAINTS =
            new PathConstraints(2.0, 2.5, Math.PI * 1.5, Math.PI * 2.5);
    private final AutoContext context;

    public TeleopAssistRoutines(AutoContext context) {
        this.context = context;
    }

    /** Press once: pathfind to a known scoring pose, then prepare the mechanism. */
    public Command driveToScoringPose(Pose2d blueAlliancePose) {
        return AutoBuilder.pathfindToPoseFlipped(blueAlliancePose, ASSIST_CONSTRAINTS)
                .alongWith(context.mechanisms.prepareToScore())
                .withName("AssistDriveToScore");
    }

    /** Hold: preserve driver translation while a controller owns only the desired heading. */
    public Command faceHeadingWhileDriving(
            DoubleSupplier driverX,
            DoubleSupplier driverY,
            Supplier<Rotation2d> desiredHeading) {
        PIDController heading = new PIDController(4.0, 0.0, 0.15);
        heading.enableContinuousInput(-Math.PI, Math.PI);
        return Commands.run(
                        () -> {
                            double omega = heading.calculate(
                                    context.drive.pose().getRotation().getRadians(),
                                    desiredHeading.get().getRadians());
                            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    driverX.getAsDouble(),
                                    driverY.getAsDouble(),
                                    omega,
                                    context.drive.pose().getRotation());
                            context.drive.driveRobotRelative(speeds);
                        },
                        context.drive)
                .finallyDo(interrupted -> context.drive.driveRobotRelative(new ChassisSpeeds()))
                .withName("AssistFaceHeading");
    }

    /** Press/hold: sample localization and the live target once, then follow a short generated path. */
    public Command driveToLiveTarget(Supplier<Pose2d> target) {
        return Commands.defer(
                        () -> AutoBuilder.followPath(shortPath(context.drive.pose(), target.get())),
                        Set.of(context.drive))
                .withName("AssistDynamicPath");
    }

    /** One-button mini-auto: align, score if still holding a piece, and stow. */
    public Command scoreAssist(Pose2d blueAlliancePose) {
        return Commands.sequence(
                        driveToScoringPose(blueAlliancePose),
                        Commands.either(
                                context.mechanisms.score(),
                                Commands.none(),
                                context.state::hasGamePiece),
                        context.mechanisms.stow())
                .withName("AssistScoreRoutine");
    }

    private static PathPlannerPath shortPath(Pose2d start, Pose2d goal) {
        Rotation2d tangent = goal.getTranslation().minus(start.getTranslation()).getAngle();
        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(start.getTranslation(), tangent),
                        new Pose2d(goal.getTranslation(), tangent)),
                ASSIST_CONSTRAINTS,
                new IdealStartingState(0.0, start.getRotation()),
                new GoalEndState(0.0, goal.getRotation()));
        path.preventFlipping = true;
        return path;
    }
}
