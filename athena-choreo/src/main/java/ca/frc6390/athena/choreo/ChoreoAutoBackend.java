package ca.frc6390.athena.choreo;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.auto.ChoreoAutoFactory;
import ca.frc6390.athena.core.auto.ChoreoBinding;
import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.TrajectorySample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ChoreoAutoBackend implements AutoBackend {
    private volatile ChoreoAutoFactory factory;
    private volatile AutoFactory autoFactory;
    private volatile Runnable stopFollower;
    private final Map<String, Supplier<Command>> namedCommandSuppliers = new ConcurrentHashMap<>();

    @Override
    public boolean supports(RobotAuto.AutoSource source) {
        return source == RobotAuto.AutoSource.CHOREO;
    }

    @Override
    public boolean registerNamedCommand(String id, Supplier<Command> supplier) {
        if (id == null || id.isBlank() || supplier == null) {
            return false;
        }
        namedCommandSuppliers.put(id, supplier);
        AutoFactory currentFactory = autoFactory;
        if (currentFactory != null) {
            currentFactory.bind(id, Commands.deferredProxy(supplier));
        }
        return true;
    }

    @Override
    public Optional<Command> buildAuto(RobotAuto.AutoSource source, String reference) {
        if (source != RobotAuto.AutoSource.CHOREO) {
            return Optional.empty();
        }
        AutoFactory currentFactory = autoFactory;
        if (currentFactory == null) {
            DriverStation.reportWarning(
                    "Choreo backend not configured; call RobotLocalization.configureChoreo(...) before building autos.",
                    false);
            return Optional.of(Commands.none());
        }
        TrajectoryRef ref = TrajectoryRef.parse(reference);
        AutoRoutine routine = currentFactory.newRoutine(reference);
        OptionalInt splitIndex = ref.splitIndex();
        AutoTrajectory trajectory = splitIndex.isPresent()
                ? routine.trajectory(ref.name(), splitIndex.getAsInt())
                : routine.trajectory(ref.name());
        routine.active().onTrue(trajectory.cmd());
        Command runRoutine = routine.cmd(trajectory.done()::getAsBoolean);
        Command stop = Commands.runOnce(() -> {
            if (stopFollower != null) {
                stopFollower.run();
            }
        });
        Command reset = splitIndex.isPresent()
                ? currentFactory.resetOdometry(ref.name(), splitIndex.getAsInt())
                : currentFactory.resetOdometry(ref.name());
        return Optional.of(Commands.sequence(reset, runRoutine, stop));
    }

    @Override
    @SuppressWarnings("unchecked")
    public Optional<ChoreoAutoFactory> createChoreoFactory(ChoreoBinding binding) {
        if (binding == null) {
            return Optional.empty();
        }

        Consumer<TrajectorySample<?>> consumer = sample -> {
            ChassisSpeeds desiredSpeeds = sample.getChassisSpeeds();
            binding.follower().accept(sample.getPose(), desiredSpeeds);
        };

        AutoFactory autoFactory = ChoreoBridge.createFactory(
                binding.poseSupplier(),
                binding.poseResetter(),
                (Consumer) consumer,
                binding.mirrorForAlliance(),
                binding.drivetrain());

        ChoreoAutoFactory createdFactory = new ChoreoAutoFactory() {
            @Override
            public Command trajectoryCmd(String trajectoryName) {
                return autoFactory.trajectoryCmd(trajectoryName);
            }

            @Override
            public Command resetOdometry(String trajectoryName) {
                return autoFactory.resetOdometry(trajectoryName);
            }
        };

        factory = createdFactory;
        this.autoFactory = autoFactory;
        stopFollower = () -> binding.follower().accept(binding.poseSupplier().get(), new ChassisSpeeds());
        bindNamedCommands(autoFactory);
        return Optional.of(createdFactory);
    }

    @Override
    public Optional<List<Pose2d>> getAutoPoses(RobotAuto.AutoSource source, String reference) {
        if (source != RobotAuto.AutoSource.CHOREO || reference == null || reference.isBlank()) {
            return Optional.empty();
        }
        TrajectoryRef ref = TrajectoryRef.parse(reference);
        Optional<? extends Trajectory<?>> trajectoryOpt = Choreo.loadTrajectory(ref.name());
        if (trajectoryOpt.isEmpty()) {
            return Optional.empty();
        }
        Trajectory<?> trajectory = trajectoryOpt.get();
        if (ref.splitIndex().isPresent()) {
            trajectory = trajectory.getSplit(ref.splitIndex().getAsInt()).orElse(null);
        }
        if (trajectory == null) {
            return Optional.empty();
        }
        Pose2d[] poses = trajectory.getPoses();
        return poses != null ? Optional.of(Arrays.asList(poses)) : Optional.empty();
    }

    private void bindNamedCommands(AutoFactory factory) {
        for (Map.Entry<String, Supplier<Command>> entry : namedCommandSuppliers.entrySet()) {
            factory.bind(entry.getKey(), Commands.deferredProxy(entry.getValue()));
        }
    }

    private record TrajectoryRef(String name, OptionalInt splitIndex) {
        private static TrajectoryRef parse(String reference) {
            if (reference == null) {
                return new TrajectoryRef("", OptionalInt.empty());
            }
            int dot = reference.lastIndexOf('.');
            if (dot <= 0 || dot == reference.length() - 1) {
                return new TrajectoryRef(reference, OptionalInt.empty());
            }
            String maybeIndex = reference.substring(dot + 1);
            try {
                int index = Integer.parseInt(maybeIndex);
                return new TrajectoryRef(reference.substring(0, dot), OptionalInt.of(index));
            } catch (NumberFormatException ex) {
                return new TrajectoryRef(reference, OptionalInt.empty());
            }
        }
    }
}
