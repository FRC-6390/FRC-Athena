package ca.frc6390.athena.vendor.choreo;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
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
import java.time.Duration;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
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
    private final Map<String, CompletableFuture<Optional<Trajectory<? extends TrajectorySample<?>>>>> trajectoryCache =
            new ConcurrentHashMap<>();
    private final Map<PreviewKey, CompletableFuture<PreparedPath>> prepared = new ConcurrentHashMap<>();
    private final ThreadPoolExecutor loader = new ThreadPoolExecutor(1, 1, 0L, TimeUnit.MILLISECONDS,
            new ArrayBlockingQueue<>(800), runnable -> {
                Thread thread = new Thread(runnable, "athena-choreo-loader");
                thread.setDaemon(true);
                thread.setPriority(Thread.MIN_PRIORITY);
                return thread;
            });
    private final AtomicLong revision = new AtomicLong();
    private final NativeRuntime runtime = new NativeRuntime();
    private volatile List<String> pathNameCache;
    private volatile CompletableFuture<List<String>> namesLoading;
    private volatile boolean closed;

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
    /** Explicit blocking loader for setup/tools. Robot execution and previews only inspect prepared results. */
    public Optional<Trajectory<? extends TrajectorySample<?>>> trajectory(String pathName) {
        return loading(pathName).join();
    }

    private CompletableFuture<Optional<Trajectory<? extends TrajectorySample<?>>>> loading(String pathName) {
        return trajectoryCache.computeIfAbsent(normalize(pathName), name -> {
            if (trajectoryCache.size() >= 256) throw new IllegalStateException("Too many prepared Choreo trajectories.");
            return submit(() -> client.loadTrajectory(name));
        });
    }

    private <T> CompletableFuture<T> submit(Supplier<T> work) {
        CompletableFuture<T> result = new CompletableFuture<>();
        if (closed) {
            result.completeExceptionally(new IllegalStateException("Choreo provider closed."));
            return result;
        }
        try {
            loader.execute(() -> {
                try { result.complete(work.get()); }
                catch (Throwable failure) { result.completeExceptionally(failure); }
                finally { revision.incrementAndGet(); }
            });
        } catch (java.util.concurrent.RejectedExecutionException full) {
            result.completeExceptionally(new IllegalStateException("Choreo preparation queue full or closed.", full));
            revision.incrementAndGet();
        }
        return result;
    }

    public synchronized List<String> pathNames() {
        List<String> names = pathNameCache;
        if (names == null && namesLoading == null) {
            namesLoading = submit(() -> List.copyOf(client.trajectoryNames()));
        }
        if (names == null && namesLoading.isDone()) pathNameCache = names = namesLoading.join();
        return names == null ? List.of() : names;
    }

    public List<String> markerNames(String pathName) {
        CompletableFuture<Optional<Trajectory<? extends TrajectorySample<?>>>> value = loading(pathName);
        return (value.isDone() ? value.join() : Optional.<Trajectory<? extends TrajectorySample<?>>>empty())
                .map(trajectory -> trajectory.events().stream()
                .map(marker -> marker.event).distinct().toList()).orElseGet(List::of);
    }

    @Override public void prepare(PathAction path) {
        preparation(path, mirrorForAlliance.getAsBoolean());
    }

    private CompletableFuture<PreparedPath> preparation(PathAction path, boolean mirrored) {
        CompletableFuture<Optional<Trajectory<? extends TrajectorySample<?>>>> loaded = loading(path.name());
        return prepared.computeIfAbsent(new PreviewKey(path, mirrored), key -> {
            if (prepared.size() >= 512) throw new IllegalStateException("Too many prepared Choreo previews.");
            return submit(() -> {
                Trajectory<? extends TrajectorySample<?>> selected = selected(path, loaded.join());
                return new PreparedPath(selected, mirrored, buildPreview(path, selected, mirrored));
            });
        });
    }

    /** Explicit off-loop setup/test fence. It never runs from a path or preview callback. */
    public boolean awaitPrepared(Duration timeout) throws InterruptedException {
        long end = System.nanoTime() + timeout.toNanos();
        List<CompletableFuture<?>> pending = new java.util.ArrayList<>(trajectoryCache.values());
        pending.addAll(prepared.values());
        if (namesLoading != null) pending.add(namesLoading);
        for (CompletableFuture<?> future : pending) {
            try { future.get(Math.max(0, end - System.nanoTime()), TimeUnit.NANOSECONDS); }
            catch (java.util.concurrent.TimeoutException timedOut) { return false; }
            catch (java.util.concurrent.ExecutionException failed) { throw new CompletionException(failed.getCause()); }
        }
        return true;
    }

    @Override public void close() {
        closed = true;
        java.util.concurrent.CancellationException closed = new java.util.concurrent.CancellationException("Choreo provider closed.");
        trajectoryCache.values().forEach(value -> value.completeExceptionally(closed));
        prepared.values().forEach(value -> value.completeExceptionally(closed));
        if (namesLoading != null) namesLoading.completeExceptionally(closed);
        loader.shutdownNow();
    }

    @Override public PathRuntime runtime() {
        requireExecutionConfiguration();
        return runtime;
    }

    @Override public Optional<PathPreview> preview(PathAction path) {
        Objects.requireNonNull(path, "path");
        if (!KEY.equals(path.source())) return Optional.empty();
        CompletableFuture<PreparedPath> result = preparation(path, mirrorForAlliance.getAsBoolean());
        return result.isDone() ? Optional.of(result.join().preview()) : Optional.empty();
    }

    private PathPreview buildPreview(PathAction path,
            Trajectory<? extends TrajectorySample<?>> value, boolean mirrored) {
        if (mirrored) value = value.flipped();
        Trajectory<? extends TrajectorySample<?>> previewTrajectory = value;
        List<PathPreview.Pose> poses = Arrays.stream(previewTrajectory.getPoses())
                .map(pose -> new PathPreview.Pose(
                        Meters.of(pose.getX()), Meters.of(pose.getY()), Radians.of(pose.getRotation().getRadians())))
                .toList();
        List<PathPreview.Event> events = previewTrajectory.events().stream()
                .filter(marker -> path.markers().containsKey(marker.event))
                .map(marker -> previewTrajectory.sampleAt(marker.timestamp, false)
                        .map(sample -> new PathPreview.Event(
                                marker.event,
                                Seconds.of(marker.timestamp),
                                new PathPreview.Pose(
                                        Meters.of(sample.getPose().getX()), Meters.of(sample.getPose().getY()),
                                        Radians.of(sample.getPose().getRotation().getRadians()))))
                        .orElse(null))
                .filter(Objects::nonNull)
                .toList();
        return new PathPreview(path.key(), poses, events);
    }

    @Override public long previewRevision() { return 2L * revision.get() + (mirrorForAlliance.getAsBoolean() ? 1L : 0L); }

    private void requireExecutionConfiguration() {
        if (follower == null && (pose == null || resetPose == null || followSample == null)) {
            throw new IllegalStateException("Choreo execution requires a kinematics path follower.");
        }
    }

    private Trajectory<? extends TrajectorySample<?>> selected(PathAction path,
            Optional<Trajectory<? extends TrajectorySample<?>>> loadedValue) {
        Trajectory<? extends TrajectorySample<?>> loaded = loadedValue
                .orElseThrow(() -> new IllegalArgumentException("Missing Choreo trajectory '" + path.name() + "'."));
        if (path.splitIndex() < 0) return loaded;
        return loaded.getSplit(path.splitIndex()).orElseThrow(() -> new IllegalArgumentException(
                "Missing split " + path.splitIndex() + " in Choreo trajectory '" + path.name() + "'."));
    }

    private record PreviewKey(PathAction path, boolean mirrored) { }
    private record PreparedPath(Trajectory<? extends TrajectorySample<?>> trajectory,
                                boolean mirrored, PathPreview preview) { }

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
            // Runtime state is keyed by the PathAction instance, and one follower drives one robot.
            // Running the same PathAction in two places at once would silently share and then
            // corrupt that state, so it is rejected (the scheduler reports it as a lease fault).
            if (active.containsKey(path)) {
                throw new IllegalStateException("Choreo path '" + path.key()
                        + "' is already running in another action; the same PathAction cannot run twice"
                        + " concurrently. Cancel the other action first or use a separate path.");
            }
            ActivePath state = new ActivePath(preparation(path, mirrorForAlliance.getAsBoolean()));
            active.put(path, state);
            try {
                startWhenPrepared(path, state, context);
            } catch (RuntimeException | Error failure) {
                active.remove(path);
                throw failure;
            }
        }

        private boolean startWhenPrepared(PathAction path, ActivePath state,
                ca.frc6390.athena.mechanism.core.MechanismContext context) {
            if (state.trajectory != null) return true;
            if (!state.preparation.isDone()) return false;
            PreparedPath ready = state.preparation.join();
            state.trajectory = ready.trajectory();
            state.mirrored = ready.mirrored();
            state.startedAt = context.timeInStateSeconds();
            if (follower != null) follower.reset();
            if (path.resetsOdometry()) state.trajectory.getInitialPose(state.mirrored)
                    .map(ChoreoPathProvider.this::resetAction)
                    .ifPresent(action -> state.activeMarkers.put("@resetOdometry", action));
            return true;
        }

        @Override public void execute(PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context) {
            ActivePath state = requireActive(path);
            if (!startWhenPrepared(path, state, context)) return;
            if (path.onlyResetsPose()) return;
            double time = Math.max(0.0, context.timeInStateSeconds() - state.startedAt);
            state.trajectory.sampleAt(time, state.mirrored).ifPresent(sample -> {
                if (!(sample instanceof SwerveSample swerve))
                    throw new IllegalStateException("Choreo path '" + path.name() + "' is not a swerve trajectory.");
                // The follower's PID runs in output(), after this cycle's markers (including the
                // odometry reset) have been applied, so the first sample never uses a stale pose.
                state.sample = swerve;
                state.sampleDtSeconds = context.dtSeconds();
                state.followPending = true;
            });
            for (var marker : state.trajectory.events()) {
                if (marker.timestamp <= time && state.triggeredMarkers.add(marker)) {
                    Action action = path.markers().get(marker.event);
                    if (action != null) state.activeMarkers.put(marker.event, action);
                }
            }
        }

        @Override public Action output(PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context) {
            if (path.onlyResetsPose()) return null;
            ActivePath state = requireActive(path);
            if (state.followPending && state.sample != null) {
                state.followPending = false;
                state.output = followAction(state.sample, state.sampleDtSeconds);
            }
            return state.output;
        }

        @Override public void markerCompleted(PathAction path, String marker) {
            ActivePath state = active.get(path);
            if (state != null) state.activeMarkers.remove(marker);
        }

        @Override public java.util.Collection<String> pendingMarkers(PathAction path) {
            ActivePath state = active.get(path);
            if (state == null) return null;
            if (state.trajectory == null) return null;
            Set<String> pending = new java.util.LinkedHashSet<>();
            for (var marker : state.trajectory.events()) {
                if (!state.triggeredMarkers.contains(marker)) pending.add(marker.event);
            }
            return pending;
        }

        @Override public List<?> ownership(PathAction path) {
            return follower == null || path.onlyResetsPose() ? List.of() : follower.ownership();
        }

        @Override public Map<String, Action> activeMarkers(
                PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context) {
            return java.util.Collections.unmodifiableMap(
                    new java.util.LinkedHashMap<>(requireActive(path).activeMarkers));
        }

        @Override public boolean isFinished(PathAction path, ca.frc6390.athena.mechanism.core.MechanismContext context) {
            ActivePath state = requireActive(path);
            if (state.trajectory == null) return false;
            if (path.onlyResetsPose()) {
                if (!state.finalSampleEvaluated) {
                    state.finalSampleEvaluated = true;
                    return false;
                }
                return true;
            }
            if (context.timeInStateSeconds() - state.startedAt < state.trajectory.getTotalTime()) return false;
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
                RobotVelocity.field(MetersPerSecond.of(sample.vx), MetersPerSecond.of(sample.vy), RadiansPerSecond.of(sample.omega))), dtSeconds);
    }

    private static final class ActivePath {
        private final CompletableFuture<PreparedPath> preparation;
        private Trajectory<? extends TrajectorySample<?>> trajectory;
        private boolean mirrored;
        private double startedAt;
        private final Set<Object> triggeredMarkers = new HashSet<>();
        private final Map<String, Action> activeMarkers = new java.util.LinkedHashMap<>();
        private Action output;
        private SwerveSample sample;
        private double sampleDtSeconds;
        private boolean followPending;
        private boolean finalSampleEvaluated;
        private ActivePath(CompletableFuture<PreparedPath> preparation) {
            this.preparation = preparation;
        }
    }
}
