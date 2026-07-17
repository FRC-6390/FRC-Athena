package ca.frc6390.athena.vendor.choreo;

import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.auto.PathPreview;
import ca.frc6390.athena.drivetrain.swerve.FollowerBackend;
import ca.frc6390.athena.drivetrain.swerve.SwervePathFollower;
import ca.frc6390.athena.drivetrain.swerve.SwervePathSample;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Arrays;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/** Native Athena Choreo provider: trajectories execute as ordinary PathActions. */
public final class ChoreoPathProvider implements PathProvider {
    public static final String KEY = "choreo";
    private final ChoreoClient client;
    private final Supplier<Pose2d> pose;
    private final Function<Pose2d, Action> resetPose;
    private final Function<SwerveSample, Action> followSample;
    private final SwervePathFollower follower;
    private final BooleanSupplier mirrorForAlliance;
    private final Map<String, Optional<Trajectory<? extends TrajectorySample<?>>>> trajectoryCache =
            new ConcurrentHashMap<>();
    private final NativeRuntime runtime = new NativeRuntime();
    private volatile List<String> pathNameCache;

    /** Loader-only provider. Configure {@link #swerve} before executing paths. */
    public ChoreoPathProvider() {
        this(new ChoreoLibClient(), null, null, null, null, () -> false);
    }

    ChoreoPathProvider(
            ChoreoClient client,
            Supplier<Pose2d> pose,
            Function<Pose2d, Action> resetPose,
            Function<SwerveSample, Action> followSample,
            BooleanSupplier mirrorForAlliance) {
        this(client, pose, resetPose, followSample, null, mirrorForAlliance);
    }

    ChoreoPathProvider(
            ChoreoClient client,
            SwervePathFollower follower,
            BooleanSupplier mirrorForAlliance) {
        this(client, null, null, null, follower, mirrorForAlliance);
    }

    private ChoreoPathProvider(
            ChoreoClient client,
            Supplier<Pose2d> pose,
            Function<Pose2d, Action> resetPose,
            Function<SwerveSample, Action> followSample,
            SwervePathFollower follower,
            BooleanSupplier mirrorForAlliance) {
        this.client = Objects.requireNonNull(client, "client");
        this.pose = pose;
        this.resetPose = resetPose;
        this.followSample = followSample;
        this.follower = follower;
        this.mirrorForAlliance = mirrorForAlliance == null ? () -> false : mirrorForAlliance;
    }

    /** Binds Choreo loading to a kinematics-owned follower. */
    public static ChoreoPathProvider swerve(
            SwervePathFollower follower, BooleanSupplier mirrorForAlliance) {
        SwervePathFollower safeFollower = Objects.requireNonNull(follower, "follower");
        if (safeFollower.backend() != FollowerBackend.CHOREO) {
            throw new IllegalArgumentException("Choreo requires a CHOREO follower backend.");
        }
        return new ChoreoPathProvider(new ChoreoLibClient(), safeFollower, mirrorForAlliance);
    }

    @Override public String source() { return KEY; }
    public Optional<Trajectory<? extends TrajectorySample<?>>> trajectory(String pathName) {
        return trajectoryCache.computeIfAbsent(normalize(pathName), client::loadTrajectory);
    }

    public List<String> pathNames() {
        List<String> names = pathNameCache;
        if (names == null) { names = client.trajectoryNames(); pathNameCache = names; }
        return names;
    }

    public List<String> markerNames(String pathName) {
        return trajectory(pathName).map(value -> value.events().stream()
                .map(marker -> marker.event).distinct().toList()).orElseGet(List::of);
    }

    @Override public PathRuntime runtime() {
        requireExecutionConfiguration();
        return runtime;
    }

    @Override public Optional<PathPreview> preview(PathAction path) {
        Objects.requireNonNull(path, "path");
        if (!KEY.equals(path.source())) return Optional.empty();
        Trajectory<? extends TrajectorySample<?>> value = selected(path);
        if (mirrorForAlliance.getAsBoolean()) value = value.flipped();
        Trajectory<? extends TrajectorySample<?>> previewTrajectory = value;
        List<PathPreview.Pose> poses = Arrays.stream(previewTrajectory.getPoses())
                .map(pose -> new PathPreview.Pose(
                        pose.getX(), pose.getY(), pose.getRotation().getRadians()))
                .toList();
        List<PathPreview.Event> events = previewTrajectory.events().stream()
                .filter(marker -> path.markers().containsKey(marker.event))
                .map(marker -> previewTrajectory.sampleAt(marker.timestamp, false)
                        .map(sample -> new PathPreview.Event(
                                marker.event,
                                marker.timestamp,
                                new PathPreview.Pose(
                                        sample.getPose().getX(), sample.getPose().getY(),
                                        sample.getPose().getRotation().getRadians())))
                        .orElse(null))
                .filter(Objects::nonNull)
                .toList();
        return Optional.of(new PathPreview(path.key(), poses, events));
    }

    @Override public long previewRevision() { return mirrorForAlliance.getAsBoolean() ? 1L : 0L; }

    private void requireExecutionConfiguration() {
        if (follower == null && (pose == null || resetPose == null || followSample == null)) {
            throw new IllegalStateException("Choreo execution requires a kinematics path follower.");
        }
    }

    private Trajectory<? extends TrajectorySample<?>> selected(PathAction path) {
        Trajectory<? extends TrajectorySample<?>> loaded = trajectory(path.name())
                .orElseThrow(() -> new IllegalArgumentException("Missing Choreo trajectory '" + path.name() + "'."));
        if (path.splitIndex() < 0) return loaded;
        return loaded.getSplit(path.splitIndex()).orElseThrow(() -> new IllegalArgumentException(
                "Missing split " + path.splitIndex() + " in Choreo trajectory '" + path.name() + "'."));
    }

    private static String normalize(String name) {
        return name == null || name.isBlank() ? "default" : name.trim();
    }

    interface ChoreoClient {
        Optional<Trajectory<? extends TrajectorySample<?>>> loadTrajectory(String name);
        List<String> trajectoryNames();
    }

    private static final class ChoreoLibClient implements ChoreoClient {
        @Override public Optional<Trajectory<? extends TrajectorySample<?>>> loadTrajectory(String name) {
            @SuppressWarnings({"rawtypes", "unchecked"})
            Optional<Trajectory<? extends TrajectorySample<?>>> loaded =
                    (Optional) Choreo.loadTrajectory(name);
            return loaded;
        }
        @Override public List<String> trajectoryNames() { return List.copyOf(Arrays.asList(Choreo.availableTrajectories())); }
    }

    private final class NativeRuntime implements PathRuntime {
        private final Map<PathAction, ActivePath> active = new IdentityHashMap<>();

        @Override public void initialize(PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context) {
            Trajectory<? extends TrajectorySample<?>> trajectory = selected(path);
            boolean mirrored = mirrorForAlliance.getAsBoolean();
            ActivePath state = new ActivePath(trajectory, mirrored);
            active.put(path, state);
            if (follower != null) follower.reset();
            if (path.resetsOdometry()) trajectory.getInitialPose(mirrored)
                    .map(ChoreoPathProvider.this::resetAction)
                    .ifPresent(action -> state.activeMarkers.put("@resetOdometry", action));
        }

        @Override public void execute(PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context) {
            ActivePath state = requireActive(path);
            double time = Math.max(0.0, context.timeInStateSeconds());
            state.trajectory.sampleAt(time, state.mirrored).ifPresent(sample -> {
                if (!(sample instanceof SwerveSample swerve))
                    throw new IllegalStateException("Choreo path '" + path.name() + "' is not a swerve trajectory.");
                state.output = followAction(swerve, context.dtSeconds());
            });
            for (var marker : state.trajectory.events()) {
                if (marker.timestamp <= time && state.triggeredMarkers.add(marker)) {
                    Action action = path.markers().get(marker.event);
                    if (action != null) state.activeMarkers.put(marker.event, action);
                }
            }
        }

        @Override public Action output(PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context) {
            return requireActive(path).output;
        }

        @Override public Map<String, Action> activeMarkers(
                PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context) {
            return java.util.Collections.unmodifiableMap(
                    new java.util.LinkedHashMap<>(requireActive(path).activeMarkers));
        }

        @Override public boolean isFinished(PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context) {
            ActivePath state = requireActive(path);
            if (context.timeInStateSeconds() < state.trajectory.getTotalTime()) return false;
            if (!state.finalSampleEvaluated) {
                state.finalSampleEvaluated = true;
                return false;
            }
            return true;
        }

        @Override public void end(PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context, boolean interrupted) {
            active.remove(path);
            if (follower != null) follower.stop();
        }

        private ActivePath requireActive(PathAction path) {
            ActivePath value = active.get(path);
            if (value == null) throw new IllegalStateException("Choreo path was not initialized: " + path.key());
            return value;
        }
    }

    private Action resetAction(Pose2d initialPose) {
        return follower == null ? resetPose.apply(initialPose) : follower.resetPose(initialPose);
    }

    private Action followAction(SwerveSample sample, double dtSeconds) {
        if (follower == null) {
            return Objects.requireNonNull(followSample.apply(sample), "followSample returned null");
        }
        return follower.follow(new SwervePathSample(
                sample.getPose(),
                RobotVelocity.field(sample.vx, sample.vy, sample.omega)), dtSeconds);
    }

    private static final class ActivePath {
        private final Trajectory<? extends TrajectorySample<?>> trajectory;
        private final boolean mirrored;
        private final Set<Object> triggeredMarkers = new HashSet<>();
        private final Map<String, Action> activeMarkers = new java.util.LinkedHashMap<>();
        private Action output;
        private boolean finalSampleEvaluated;
        private ActivePath(Trajectory<? extends TrajectorySample<?>> trajectory, boolean mirrored) {
            this.trajectory = trajectory; this.mirrored = mirrored;
        }
    }
}
