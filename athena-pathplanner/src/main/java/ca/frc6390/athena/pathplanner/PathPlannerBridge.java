package ca.frc6390.athena.pathplanner;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Minimal PathPlanner bridge for the new vendordep system.
 */
public final class PathPlannerBridge {
    private PathPlannerBridge() {
    }

    public static void configure(
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> poseReset,
            Supplier<ChassisSpeeds> speedsSupplier,
            BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
            PathFollowingController controller,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlip,
            Subsystem... requirements) {
        AutoBuilder.configure(
                poseSupplier,
                poseReset,
                speedsSupplier,
                output,
                controller,
                robotConfig,
                shouldFlip,
                requirements);
    }

    public static void configureSimple(
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> poseReset,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<ChassisSpeeds> output,
            PathFollowingController controller,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlip,
            Subsystem... requirements) {
        AutoBuilder.configure(
                poseSupplier,
                poseReset,
                speedsSupplier,
                output,
                controller,
                robotConfig,
                shouldFlip,
                requirements);
    }

    public static Command buildAuto(String name) {
        return new PathPlannerAuto(name);
    }

    public static PathPlannerPath loadPath(String name) {
        return PathPlannerPath.fromPathFile(name);
    }

    public static Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    public static Command pathfindToPose(Pose2d target, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(target, constraints);
    }

    public static SendableChooser<Command> buildAutoChooser() {
        return AutoBuilder.buildAutoChooser();
    }

    public static SendableChooser<Command> buildAutoChooser(String defaultAuto) {
        return AutoBuilder.buildAutoChooser(defaultAuto);
    }

    public static void registerNamedCommand(String name, Command command) {
        NamedCommands.registerCommand(name, command);
    }
}
