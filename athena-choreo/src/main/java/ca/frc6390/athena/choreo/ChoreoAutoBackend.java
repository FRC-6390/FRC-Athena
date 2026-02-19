package ca.frc6390.athena.choreo;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BiConsumer;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ChoreoAutoBackend implements AutoBackend {
    private static final double POSE_SETTLE_TRANSLATION_TOLERANCE_METERS = 0.10;
    private static final double POSE_SETTLE_HEADING_TOLERANCE_DEG = 2.0;
    private static final double POSE_SETTLE_TIMEOUT_SECONDS = 0.75;

    private volatile ChoreoAutoFactory factory;
    private volatile AutoFactory autoFactory;
    private volatile Runnable stopFollower;
    private volatile Supplier<Pose2d> poseSupplier;
    private volatile BiConsumer<Pose2d, ChassisSpeeds> follower;
    private volatile Subsystem followerSubsystem;
    private final Map<String, Supplier<Command>> namedCommandSuppliers = new ConcurrentHashMap<>();

    @Override
    public boolean supports(RobotAuto.AutoSource source) {
        return source == RobotAuto.AutoSource.CHOREO;
    }

    @Override
    public int priority(RobotAuto.AutoSource source) {
        return source == RobotAuto.AutoSource.CHOREO ? 100 : 0;
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
        Command resetRaw = splitIndex.isPresent()
                ? currentFactory.resetOdometry(ref.name(), splitIndex.getAsInt())
                : currentFactory.resetOdometry(ref.name());
        Command reset = Commands.either(
                resetRaw,
                Commands.runOnce(() -> DriverStation.reportWarning(
                        "Skipping Choreo odometry reset for \"" + reference
                                + "\" because alliance is unknown.",
                        false)),
                () -> DriverStation.getAlliance().isPresent());
        boolean shouldResetOdometry = splitIndex.isEmpty() || splitIndex.getAsInt() == 0;
        AtomicBoolean trajectoryFinished = new AtomicBoolean(false);
        Command startCommand = shouldResetOdometry
                ? Commands.sequence(reset, trajectory.cmd())
                : trajectory.cmd();
        routine.active()
                .onTrue(startCommand.finallyDo(interrupted -> trajectoryFinished.set(true)));
        Command runRoutine = routine.cmd(trajectoryFinished::get);
        Command poseSettle = buildPoseSettleCommand(reference, trajectory);
        Command stop = Commands.runOnce(() -> {
            if (stopFollower != null) {
                stopFollower.run();
            }
        });
        return Optional.of(Commands.sequence(runRoutine, poseSettle, stop));
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
        this.poseSupplier = binding.poseSupplier();
        this.follower = binding.follower();
        this.followerSubsystem = binding.drivetrain();
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

    private Command buildPoseSettleCommand(String reference, AutoTrajectory trajectory) {
        Supplier<Pose2d> currentPoseSupplier = poseSupplier;
        BiConsumer<Pose2d, ChassisSpeeds> follow = follower;
        Subsystem requirement = followerSubsystem;
        if (currentPoseSupplier == null || follow == null || requirement == null) {
            return Commands.none();
        }
        return Commands.defer(() -> {
            Optional<Pose2d> targetPoseOpt = trajectory.getFinalPose();
            if (targetPoseOpt.isEmpty()) {
                return Commands.none();
            }
            Pose2d targetPose = targetPoseOpt.get();
            AtomicBoolean timedOut = new AtomicBoolean(false);
            Command settle = Commands.run(
                            () -> {
                                follow.accept(targetPose, new ChassisSpeeds());
                            },
                            requirement)
                    .until(() -> {
                        Pose2d current = currentPoseSupplier.get();
                        double translationErrorMeters =
                                current.getTranslation().getDistance(targetPose.getTranslation());
                        double headingErrorDeg = Math.abs(MathUtil.inputModulus(
                                targetPose.getRotation().getDegrees() - current.getRotation().getDegrees(),
                                -180.0,
                                180.0));
                        return translationErrorMeters <= POSE_SETTLE_TRANSLATION_TOLERANCE_METERS
                                && headingErrorDeg <= POSE_SETTLE_HEADING_TOLERANCE_DEG;
                    })
                    .withTimeout(POSE_SETTLE_TIMEOUT_SECONDS)
                    .beforeStarting(() -> DriverStation.reportWarning(
                            "[Choreo] Pose settle start \"" + reference + "\"", false))
                    .finallyDo(interrupted -> {
                        if (interrupted) {
                            timedOut.set(true);
                        }
                        Pose2d current = currentPoseSupplier.get();
                        double translationErrorMeters =
                                current.getTranslation().getDistance(targetPose.getTranslation());
                        double headingErrorDeg = Math.abs(MathUtil.inputModulus(
                                targetPose.getRotation().getDegrees() - current.getRotation().getDegrees(),
                                -180.0,
                                180.0));
                        DriverStation.reportWarning(
                                "[Choreo] Pose settle end \"" + reference
                                        + "\" translationErrorM="
                                        + String.format(java.util.Locale.US, "%.3f", translationErrorMeters)
                                        + " errorDeg=" + String.format(java.util.Locale.US, "%.2f", headingErrorDeg)
                                        + " timeout=" + timedOut.get(),
                                false);
                    });
            return settle;
        }, Set.of(requirement));
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
