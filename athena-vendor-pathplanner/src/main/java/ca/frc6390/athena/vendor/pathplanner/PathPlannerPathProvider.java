package ca.frc6390.athena.vendor.pathplanner;

import java.util.List;
import java.util.Map;
import java.util.IdentityHashMap;
import java.time.Duration;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;
import java.util.Objects;

import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathAction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * PathPlanner-backed provider for Athena path Actions and runtimes.
 */
public final class PathPlannerPathProvider implements PathProvider {
    /**
     * Source key used by PathPlanner path Actions.
     */
    public static final String KEY = "pathplanner";

    private final PathPlannerClient client;
    private volatile List<String> pathNameCache;
    private volatile CompletableFuture<List<String>> namesLoading;
    private volatile Map<PathAction, CompletableFuture<Command>> prepared = Map.of();
    private final PathRuntime pathRuntime = new CommandPathRuntime();
    private final ThreadPoolExecutor loader = new ThreadPoolExecutor(1, 1, 0, TimeUnit.MILLISECONDS,
            new ArrayBlockingQueue<>(1), work -> {
                Thread thread = new Thread(work, "athena-pathplanner-loader");
                thread.setDaemon(true);
                thread.setPriority(Thread.MIN_PRIORITY);
                return thread;
            });
    private volatile boolean closed;

    /**
     * Creates a provider that delegates to PathPlannerLib's global AutoBuilder.
     */
    public PathPlannerPathProvider() {
        this(new AutoBuilderPathPlannerClient());
    }

    PathPlannerPathProvider(PathPlannerClient client) {
        this.client = Objects.requireNonNull(client, "client");
    }

    @Override
    public String source() {
        return KEY;
    }

    /**
     * Returns the PathPlanner auto as a path Action. It composes like any path Action: it can be placed in an
     * AutoChooser, a sequence, a binding, or a race, and the auto's command runs through {@link #runtime()}.
     * AthenaRobot preloads chooser paths after robot-init hooks. Headless runtimes call
     * RobotRuntime.preparePathsForStartup(), or explicitly preload(path), during setup. An unprepared runtime
     * request fails promptly without loading files.
     *
     * @param pathName PathPlanner auto name
     * @return PathPlanner path Action
     */
    public PathAction load(String pathName) {
        return new PathAction(KEY, normalize(pathName));
    }

    /**
     * Explicit blocking setup/tool API that loads a WPILib command from a PathPlanner auto file.
     * Robot path execution uses the startup preparation cache instead.
     *
     * @param pathName PathPlanner auto name
     * @return WPILib command
     */
    public Command command(String pathName) {
        return client.buildAuto(normalize(pathName));
    }

    /**
     * Creates an Athena path runtime that runs PathPlanner autos by path name.
     *
     * @return path runtime
     */
    public PathRuntime runtime() {
        return pathRuntime;
    }

    /**
     * Returns path names discovered by PathPlannerLib.
     *
     * @return path names
     */
    public synchronized List<String> pathNames() {
        if (pathNameCache == null && namesLoading == null) {
            namesLoading = submit(() -> List.copyOf(client.autoNames()));
        }
        if (pathNameCache == null && namesLoading.isDone()) pathNameCache = ready(namesLoading);
        return pathNameCache == null ? List.of() : pathNameCache;
    }

    /** Builds commands only during explicit startup setup, before the WPILib scheduler runs. */
    @Override public void prepareForStartup(PathAction path) { preload(path); }

    /**
     * Blocking setup/tool API. Call on the robot setup thread before periodic execution starts;
     * PathPlanner command construction updates WPILib scheduler composition state.
     */
    public void preload(PathAction path) {
        if (closed) throw new IllegalStateException("PathPlanner provider closed.");
        if (prepared.containsKey(path)) return;
        if (prepared.size() >= 512) throw new IllegalStateException("Too many prepared PathPlanner paths.");
        CompletableFuture<Command> command = new CompletableFuture<>();
        try {
            command.complete(Objects.requireNonNull(path.onlyResetsPose()
                    ? client.resetPose(path.name()) : client.buildAuto(path.name())));
        } catch (Throwable failure) {
            // A missing/bad auto faults its Action when selected, rather than taking down robot startup.
            command.completeExceptionally(failure);
        }
        synchronized (this) {
            if (closed) throw new IllegalStateException("PathPlanner provider closed.");
            Map<PathAction, CompletableFuture<Command>> updated = new IdentityHashMap<>(prepared);
            updated.put(path, command);
            prepared = java.util.Collections.unmodifiableMap(updated);
        }
    }

    @Override public void prepare(PathAction path) {
        // A runtime request cannot safely build WPILib command compositions on a worker.
        // It must already have been preloaded during setup; initialize supplies the clear error.
    }

    private CompletableFuture<Command> preparation(PathAction path) {
        // Identity lookup only; preload is explicitly a setup-only operation on the same thread.
        CompletableFuture<Command> command = prepared.get(path);
        if (command == null) throw new IllegalStateException("PathPlanner path '" + path.key()
                + "' was not preloaded. Declare it in an AutoChooser or call provider.preload(path) during setup.");
        return command;
    }

    private <T> CompletableFuture<T> submit(Supplier<T> work) {
        CompletableFuture<T> future = new CompletableFuture<>();
        if (closed) {
            future.completeExceptionally(new IllegalStateException("PathPlanner provider closed."));
            return future;
        }
        try {
            loader.execute(() -> {
                try { future.complete(Objects.requireNonNull(work.get(), "PathPlanner preparation returned null")); }
                catch (Throwable failure) { future.completeExceptionally(failure); }
            });
        } catch (java.util.concurrent.RejectedExecutionException full) {
            future.completeExceptionally(new IllegalStateException("PathPlanner preparation queue full or closed.", full));
        }
        return future;
    }

    /** Explicit off-loop setup/test fence; callbacks never wait for preparation. */
    public boolean awaitPrepared(Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        List<CompletableFuture<?>> pending = new java.util.ArrayList<>(prepared.values());
        if (namesLoading != null) pending.add(namesLoading);
        for (CompletableFuture<?> future : pending) {
            try { future.get(Math.max(0, deadline - System.nanoTime()), TimeUnit.NANOSECONDS); }
            catch (java.util.concurrent.TimeoutException timedOut) { return false; }
            catch (java.util.concurrent.ExecutionException failure) { ready(future); }
        }
        return true;
    }

    @Override public void close() {
        closed = true;
        var cancelled = new java.util.concurrent.CancellationException("PathPlanner provider closed.");
        prepared.forEach((path, value) -> value.completeExceptionally(cancelled));
        if (namesLoading != null) namesLoading.completeExceptionally(cancelled);
        loader.shutdownNow();
    }

    private static <T> T ready(CompletableFuture<T> future) {
        try { return future.join(); }
        catch (CompletionException failure) {
            if (failure.getCause() instanceof RuntimeException cause) throw cause;
            throw failure;
        }
    }

    private static String normalize(String pathName) {
        return pathName == null || pathName.isBlank() ? "default" : pathName.trim();
    }

    interface PathPlannerClient {
        Command buildAuto(String autoName);

        Command resetPose(String autoName);

        List<String> autoNames();
    }

    private final class CommandPathRuntime implements PathRuntime {
        private final Map<PathAction, ActiveCommand> activeCommands = new IdentityHashMap<>();

        @Override public void initialize(PathAction path, MechanismContext context) {
            if (activeCommands.containsKey(path)) throw new IllegalStateException("Path already running: " + path.key());
            ActiveCommand active = new ActiveCommand(preparation(path));
            activeCommands.put(path, active);
            try {
                startIfReady(active);
            } catch (RuntimeException | Error failure) {
                activeCommands.remove(path);
                if (active.command != null) {
                    try { active.command.end(true); }
                    catch (RuntimeException | Error cleanup) { if (cleanup != failure) failure.addSuppressed(cleanup); }
                }
                throw failure;
            }
        }

        @Override public void execute(PathAction path, MechanismContext context) {
            ActiveCommand active = activeCommand(path);
            if (startIfReady(active)) active.command.execute();
        }

        @Override public boolean isFinished(PathAction path, MechanismContext context) {
            ActiveCommand active = activeCommand(path);
            return active.command != null && active.command.isFinished();
        }

        @Override public void end(PathAction path, MechanismContext context, boolean interrupted) {
            ActiveCommand active = activeCommands.remove(path);
            // WPILib commands support reinitialization; keep the prepared instance for the next run.
            if (active != null && active.command != null) active.command.end(interrupted);
        }

        private boolean startIfReady(ActiveCommand active) {
            if (active.command != null) return true;
            if (!active.preparation.isDone()) return false;
            active.command = ready(active.preparation);
            // Construction/loading happened during setup; all command callbacks stay on the robot thread.
            active.command.initialize();
            return true;
        }

        private ActiveCommand activeCommand(PathAction path) {
            ActiveCommand active = activeCommands.get(path);
            if (active == null) throw new IllegalStateException("Path was not initialized: " + path.key());
            return active;
        }

        private static final class ActiveCommand {
            private final CompletableFuture<Command> preparation;
            private Command command;
            private ActiveCommand(CompletableFuture<Command> preparation) { this.preparation = preparation; }
        }
    }

    private static final class AutoBuilderPathPlannerClient implements PathPlannerClient {
        @Override
        public Command buildAuto(String autoName) {
            return AutoBuilder.buildAuto(autoName);
        }

        @Override
        public Command resetPose(String autoName) {
            Command auto = AutoBuilder.buildAuto(autoName);
            if (!(auto instanceof PathPlannerAuto pathPlannerAuto) || pathPlannerAuto.getStartingPose() == null) {
                throw new IllegalArgumentException(
                        "PathPlanner auto '" + autoName + "' does not define an initial pose.");
            }
            return AutoBuilder.resetOdom(pathPlannerAuto.getStartingPose());
        }

        @Override
        public List<String> autoNames() {
            return List.copyOf(AutoBuilder.getAllAutoNames());
        }
    }
}
