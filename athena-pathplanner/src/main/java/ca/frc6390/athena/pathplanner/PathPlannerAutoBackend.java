package ca.frc6390.athena.pathplanner;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.auto.ChoreoAutoFactory;
import ca.frc6390.athena.core.auto.ChoreoBinding;
import ca.frc6390.athena.core.auto.HolonomicDriveBinding;
import ca.frc6390.athena.core.auto.HolonomicFeedforward;
import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import choreo.auto.AutoFactory;
import choreo.trajectory.TrajectorySample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PathPlannerAutoBackend implements AutoBackend {

    @Override
    public boolean supports(RobotAuto.AutoSource source) {
        return source == RobotAuto.AutoSource.PATH_PLANNER || source == RobotAuto.AutoSource.CHOREO;
    }

    @Override
    public boolean configureHolonomic(HolonomicDriveBinding binding) {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to load PathPlanner config: " + e.getMessage(), e.getStackTrace());
            return false;
        }

        AutoBuilder.configure(
                binding.poseSupplier(),
                binding.poseResetter(),
                binding.robotRelativeSpeeds(),
                (speeds, feed) -> binding.output().accept(speeds, HolonomicFeedforward.zero()),
                new PPHolonomicDriveController(
                        toPid(binding.translationPid()),
                        toPid(binding.rotationPid())),
                config,
                () -> binding.isRedAllianceSupplier().get());

        return true;
    }

    @Override
    public Optional<Command> warmupCommand(RobotAuto.AutoSource source) {
        if (source != RobotAuto.AutoSource.PATH_PLANNER) {
            return Optional.empty();
        }
        return Optional.of(FollowPathCommand.warmupCommand());
    }

    @Override
    public boolean registerNamedCommand(String id, Supplier<Command> supplier) {
        NamedCommands.registerCommand(id, supplier.get());
        return true;
    }

    @Override
    public Optional<Command> buildAuto(RobotAuto.AutoSource source, String reference) {
        if (source == RobotAuto.AutoSource.PATH_PLANNER) {
            return Optional.of(new PathPlannerAuto(reference));
        }

        if (source == RobotAuto.AutoSource.CHOREO) {
            try {
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(reference);
                return Optional.of(AutoBuilder.followPath(path));
            } catch (FileVersionException | IOException | ParseException ex) {
                DriverStation.reportError("Could not load Choreo auto \"" + reference + "\"", ex.getStackTrace());
                return Optional.of(Commands.none());
            }
        }

        return Optional.empty();
    }

    @Override
    public Optional<ChoreoAutoFactory> createChoreoFactory(ChoreoBinding binding) {
        if (binding == null) {
            return Optional.empty();
        }

        java.util.function.Consumer<TrajectorySample<?>> consumer = sampleConsumer(binding);

        AutoFactory factory = new AutoFactory(
                binding.poseSupplier(),
                binding.poseResetter(),
                (java.util.function.Consumer) consumer,
                binding.mirrorForAlliance(),
                binding.drivetrain());

        return Optional.of(new ChoreoAutoFactory() {
            @Override
            public Command trajectoryCmd(String trajectoryName) {
                return factory.trajectoryCmd(trajectoryName);
            }

            @Override
            public Command resetOdometry(String trajectoryName) {
                return factory.resetOdometry(trajectoryName);
            }
        });
    }

    @Override
    public Optional<List<Pose2d>> getAutoPoses(RobotAuto.AutoSource source, String reference) {
        if (reference == null || reference.isBlank()) {
            return Optional.empty();
        }
        if (source == RobotAuto.AutoSource.PATH_PLANNER) {
            try {
                List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(reference);
                return Optional.of(flattenPathPoses(paths));
            } catch (IOException | ParseException ex) {
                try {
                    PathPlannerPath path = PathPlannerPath.fromPathFile(reference);
                    return Optional.of(path.getPathPoses());
                } catch (IOException | ParseException | FileVersionException pathEx) {
                    DriverStation.reportWarning(
                            "Failed to load PathPlanner auto poses for \"" + reference + "\"", pathEx.getStackTrace());
                }
            }
        }
        if (source == RobotAuto.AutoSource.CHOREO) {
            try {
                ChoreoRef ref = ChoreoRef.parse(reference);
                PathPlannerPath path = ref.index().isPresent()
                        ? PathPlannerPath.fromChoreoTrajectory(ref.name(), ref.index().getAsInt())
                        : PathPlannerPath.fromChoreoTrajectory(ref.name());
                return Optional.of(path.getPathPoses());
            } catch (IOException | ParseException | FileVersionException ex) {
                DriverStation.reportWarning(
                        "Failed to load Choreo poses for \"" + reference + "\"", ex.getStackTrace());
            }
        }
        return Optional.empty();
    }

    private static PIDConstants toPid(HolonomicPidConstants pid) {
        return new PIDConstants(pid.kP(), pid.kI(), pid.kD(), pid.iZone());
    }

    private static java.util.function.Consumer<TrajectorySample<?>> sampleConsumer(ChoreoBinding binding) {
        return sample -> {
            ChassisSpeeds desiredSpeeds = sample.getChassisSpeeds();
            binding.follower().accept(sample.getPose(), desiredSpeeds);
        };
    }

    private static List<Pose2d> flattenPathPoses(List<PathPlannerPath> paths) {
        if (paths == null || paths.isEmpty()) {
            return List.of();
        }
        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : paths) {
            if (path == null) {
                continue;
            }
            List<Pose2d> pathPoses = path.getPathPoses();
            if (pathPoses == null || pathPoses.isEmpty()) {
                continue;
            }
            if (poses.isEmpty()) {
                poses.addAll(pathPoses);
            } else {
                poses.addAll(pathPoses.subList(1, pathPoses.size()));
            }
        }
        return poses;
    }

    private record ChoreoRef(String name, OptionalInt index) {
        private static ChoreoRef parse(String reference) {
            if (reference == null) {
                return new ChoreoRef("", OptionalInt.empty());
            }
            int dot = reference.lastIndexOf('.');
            if (dot <= 0 || dot == reference.length() - 1) {
                return new ChoreoRef(reference, OptionalInt.empty());
            }
            String maybeIndex = reference.substring(dot + 1);
            try {
                int idx = Integer.parseInt(maybeIndex);
                return new ChoreoRef(reference.substring(0, dot), OptionalInt.of(idx));
            } catch (NumberFormatException ex) {
                return new ChoreoRef(reference, OptionalInt.empty());
            }
        }
    }
}
