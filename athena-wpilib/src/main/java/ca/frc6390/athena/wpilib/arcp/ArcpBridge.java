package ca.frc6390.athena.wpilib.arcp;

import ca.frc6390.athena.arcp.ArcpCycle;
import ca.frc6390.athena.arcp.ArcpPolicy;
import ca.frc6390.athena.arcp.ArcpStatus;
import ca.frc6390.athena.arcp.ArcpSink;
import ca.frc6390.athena.arcp.NativeArcpServer;
import ca.frc6390.athena.arcp.Nt4Bridge;
import ca.frc6390.athena.mechanism.persist.PersistentStore;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.replay.ReplayRecorder;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Predicate;
import java.util.function.Supplier;

/**
 * Connects a robot runtime to ARCP: builds the signal table from the mechanism telemetry schema, starts the
 * native server, forwards cycles, writes, actions, markers and replay records, and keeps the file-service
 * exports up to date.
 *
 * <p>Created by {@code AthenaRobot} at robot init and driven once per robot cycle with {@link #periodic}.
 * When the policy is disabled or the native library cannot start, the bridge does nothing and reports why.</p>
 */
public final class ArcpBridge implements AutoCloseable {

    /** File-service category names. */
    public static final String CATEGORY_REPLAYS = "replays";
    public static final String CATEGORY_STORE = "store";
    public static final String CATEGORY_MODELS = "models";
    public static final String CATEGORY_TUNING = "tuning";
    public static final String CATEGORY_STATUS = "status";

    private static final double EXPORT_PERIOD_SECONDS = 1.0;

    private final NativeArcpServer server;
    private final ArcpSignals signals;
    private final ArcpCycle cycle = new ArcpCycle();
    private final ArcpTuning tuning;
    private final ReplayRecorder recorder;
    private final Supplier<Map<String, Object>> codeDefaults;
    private final Path statusDirectory;
    private double lastExportSeconds = Double.NaN;
    private String lastFailure = "";
    private boolean closed;

    private ArcpBridge(NativeArcpServer server, ArcpSignals signals, ArcpTuning tuning, ReplayRecorder recorder,
            Supplier<Map<String, Object>> codeDefaults, Path statusDirectory) {
        this.server = server;
        this.signals = signals;
        this.tuning = tuning;
        this.recorder = recorder;
        this.codeDefaults = codeDefaults;
        this.statusDirectory = statusDirectory;
    }

    /**
     * Finds the robot's {@link ArcpPolicy} field. Any visibility is accepted; the first non-null value wins.
     * Without a field the default policy applies (ARCP enabled).
     *
     * @param robot robot instance
     * @return policy
     */
    public static ArcpPolicy discoverPolicy(Object robot) {
        for (Class<?> type = robot.getClass(); type != null && type != Object.class; type = type.getSuperclass()) {
            for (Field field : type.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers()) || field.getType() != ArcpPolicy.class) {
                    continue;
                }
                try {
                    field.setAccessible(true);
                    if (field.get(robot) instanceof ArcpPolicy policy) {
                        return policy;
                    }
                } catch (IllegalAccessException | RuntimeException exception) {
                    throw new IllegalStateException("Cannot read ARCP policy field " + field + ".", exception);
                }
            }
        }
        return ArcpPolicy.of();
    }

    /**
     * Starts ARCP for a robot. Call after the runtime, replay recorder and persistent store exist and before the
     * telemetry schema is first published.
     *
     * @param robot robot instance whose {@link ArcpPolicy} field applies
     * @param runtime robot runtime
     * @param recorder replay recorder, or null when replay is not enabled
     * @param locked lock-group condition (true while writes to the group are rejected)
     * @param store persistent store, or null when persistence is disabled
     * @param simulation true in simulation (selects simulation default directories)
     * @return bridge; {@link #enabled()} is false when ARCP is off or could not start
     */
    public static ArcpBridge attach(Object robot, RobotRuntime runtime, ReplayRecorder recorder,
            Predicate<Class<?>> locked, PersistentStore store, boolean simulation) {
        return attach(discoverPolicy(robot), runtime, recorder, locked, store, simulation);
    }

    /** As {@link #attach(Object, RobotRuntime, ReplayRecorder, Predicate, PersistentStore, boolean)} with an explicit policy. */
    public static ArcpBridge attach(ArcpPolicy policy, RobotRuntime runtime, ReplayRecorder recorder,
            Predicate<Class<?>> locked, PersistentStore store, boolean simulation) {
        Objects.requireNonNull(policy, "policy");
        Objects.requireNonNull(runtime, "runtime");
        Objects.requireNonNull(locked, "locked");
        if (!policy.enabled()) {
            return disabled();
        }
        Path storeDirectory = policy.storeDirectory().orElse(store != null ? store.directory()
                : defaultDirectory(simulation, "athena/store", "athena-sim-store"));
        Path replays = recorder != null ? recorder.directory()
                : defaultDirectory(simulation, "athena/replays", "athena-sim-replays");
        Path models = policy.modelDirectory().orElse(defaultDirectory(simulation, "athena/models", "athena-sim-models"));
        Path exports = storeDirectory.resolveSibling(storeDirectory.getFileName() + "-exports");
        Path tuningExports = exports.resolve(CATEGORY_TUNING);
        Path statusDirectory = exports.resolve(CATEGORY_STATUS);
        Path deployed = policy.deployedTuningFile().orElse(defaultDeployedTuning(simulation));

        Map<String, Path> categories = new LinkedHashMap<>();
        categories.put(CATEGORY_REPLAYS, replays);
        categories.put(CATEGORY_STORE, storeDirectory);
        categories.put(CATEGORY_MODELS, models);
        categories.put(CATEGORY_TUNING, tuningExports);
        categories.put(CATEGORY_STATUS, statusDirectory);
        for (Path directory : categories.values()) {
            try {
                Files.createDirectories(directory);
            } catch (IOException | RuntimeException exception) {
                // A category that cannot be created lists as empty; nothing else depends on it.
            }
        }

        ArcpSignals signals = new ArcpSignals(runtime.mechanismTelemetrySchema(), locked);
        NativeArcpServer server = startServer(policy, categories, signals, simulation);
        if (server == null) {
            return disabled();
        }
        ArcpTuning tuning = new ArcpTuning(ArcpTuning.readDeployed(deployed), tuningExports);
        ArcpBridge bridge = new ArcpBridge(server, signals, tuning, recorder,
                () -> runtime.telemetryPersistence().codeDefaults(), statusDirectory);
        try {
            server.publishDictionary(signals.dictionary(), List.of());
        } catch (RuntimeException failure) {
            DriverStation.reportWarning("Athena ARCP dictionary failed: " + failure.getMessage(), false);
        }
        if (recorder != null) {
            recorder.addMarkerListener((timestampSeconds, kind, message, signalId) ->
                    bridge.forwardMarker(timestampSeconds, kind, message));
            recorder.streamSink(new ArcpReplaySink(server));
        }
        return bridge;
    }

    /**
     * Starts the native server on the policy's ports. If they are taken, a real robot fails ARCP loudly with a
     * DriverStation error naming the ports; simulation logs a warning and falls back to ephemeral ports so ARCP still
     * runs (the bound ports are printed at startup).
     *
     * @return the running server, or null when ARCP could not start
     */
    private static NativeArcpServer startServer(ArcpPolicy policy, Map<String, Path> categories, ArcpSignals signals,
            boolean simulation) {
        int control = policy.controlPort();
        int realtime = policy.realtimePort();
        NativeArcpServer server;
        try {
            server = open(control, realtime, policy, categories, signals);
        } catch (UnsatisfiedLinkError failure) {
            DriverStation.reportWarning("Athena ARCP is disabled: " + failure.getMessage(), false);
            return null;
        } catch (RuntimeException failure) {
            String reason = failure.getMessage();
            if (!simulation) {
                DriverStation.reportError("Athena ARCP is disabled: could not bind TCP " + control + " or UDP "
                        + realtime + " (" + reason + "). Another program is using these ports; NT4 uses 5810. "
                        + "Stop it or change ArcpPolicy.controlPort/realtimePort.", false);
                return null;
            }
            try {
                server = open(0, 0, policy, categories, signals);
            } catch (RuntimeException | UnsatisfiedLinkError retry) {
                DriverStation.reportError("Athena ARCP is disabled: TCP " + control + " / UDP " + realtime
                        + " in use (" + reason + ") and ephemeral fallback failed (" + retry.getMessage() + ").", false);
                return null;
            }
            DriverStation.reportWarning("Athena ARCP: TCP " + control + " / UDP " + realtime + " in use (" + reason
                    + "); simulation fell back to ephemeral ports.", false);
        }
        System.out.println("ARCP listening on TCP " + server.controlPort() + " / UDP " + server.realtimePort());
        return server;
    }

    private static NativeArcpServer open(int control, int realtime, ArcpPolicy policy,
            Map<String, Path> categories, ArcpSignals signals) {
        return new NativeArcpServer(
                new NativeArcpServer.Config(control, realtime, null, policy.bandwidthBudgetBytesPerSecond(),
                        policy.mtu(), Math.max(1L, policy.keyframeInterval().toNanos() / 1_000L)),
                categories,
                signals::write,
                signals::action,
                Nt4Bridge.DISABLED);
    }

    /**
     * Installs the deployed tuning file as the store's read-only lower layer: robot store, then deployed file, then
     * code default. Call before the store is handed to the runtime.
     *
     * @param store persistent store
     * @param policy ARCP policy (selects the deployed file)
     * @param simulation true in simulation
     */
    public static void installDeployedTuning(PersistentStore store, ArcpPolicy policy, boolean simulation) {
        Path file = policy.deployedTuningFile().orElse(defaultDeployedTuning(simulation));
        store.fallback(ArcpTuning.readDeployed(file));
    }

    /** Sends a replay marker to ARCP clients. Replay signal ids do not map to ARCP signals, so markers carry 0. */
    private void forwardMarker(double timestampSeconds, int kind, String message) {
        if (server == null || closed) {
            return;
        }
        try {
            server.marker(micros(timestampSeconds), kind, message, 0L);
        } catch (RuntimeException failure) {
            report(failure);
        }
    }

    private static ArcpBridge disabled() {
        return new ArcpBridge(null, null, null, null, Map::of, null);
    }

    /** TCP control port of the running server, or -1 when disabled. */
    public int controlPort() {
        return server == null ? -1 : server.controlPort();
    }

    /** True when the native server is running. */
    public boolean enabled() {
        return server != null;
    }

    /** The sink the robot publishes through, or {@link ArcpSink#NONE} when disabled. */
    public ArcpSink sink() {
        return server == null ? ArcpSink.NONE : server;
    }

    /**
     * Called once per robot cycle: answers queued client requests, publishes changed values, and refreshes the
     * file-service exports about once a second.
     *
     * @param nowSeconds robot timestamp in seconds
     */
    public void periodic(double nowSeconds) {
        if (server == null || closed) {
            return;
        }
        try {
            server.serviceRequests();
            signals.sample(cycle);
            if (cycle.size() > 0) {
                server.publishCycle(micros(nowSeconds), cycle);
            }
        } catch (RuntimeException failure) {
            report(failure);
        }
        if (Double.isNaN(lastExportSeconds) || nowSeconds - lastExportSeconds >= EXPORT_PERIOD_SECONDS) {
            lastExportSeconds = nowSeconds;
            refreshExports();
        }
    }

    private void refreshExports() {
        try {
            tuning.refresh(signals.persistedValues(), codeDefaults.get());
            ArcpStatus status = server.status();
            StringBuilder text = new StringBuilder("# athena arcp status\n");
            text.append("arcp.sessions=").append(status.sessions()).append('\n');
            text.append("arcp.subscriptions=").append(status.subscriptions()).append('\n');
            text.append("arcp.bytesLastWindow=").append(status.bytesLastWindow()).append('\n');
            text.append("arcp.throttled=").append(status.throttled().size()).append('\n');
            text.append("arcp.signals=").append(signals.valueCount()).append('\n');
            if (recorder != null) {
                var policy = recorder.policy();
                var retention = policy.retention();
                text.append("replay.streaming=").append(recorder.streaming()).append('\n');
                text.append("replay.bufferBytes=").append(policy.bufferBytes()).append('\n');
                text.append("replay.pendingFlushes=").append(recorder.pendingFlushes()).append('\n');
                text.append("replay.droppedFlushes=").append(recorder.droppedFlushes()).append('\n');
                text.append("retention.maxTotalBytes=").append(retention.maxTotalBytes()).append('\n');
                text.append("retention.maxFiles=").append(retention.maxFiles()).append('\n');
                text.append("retention.minimumFreeDiskBytes=").append(retention.minimumFreeDiskBytes()).append('\n');
                text.append("retention.keepsMatchesOverPractice=")
                        .append(retention.keepsMatchesOverPractice()).append('\n');
                try {
                    text.append("disk.usableBytes=")
                            .append(Files.getFileStore(recorder.directory()).getUsableSpace()).append('\n');
                } catch (java.io.IOException | RuntimeException unavailable) {
                    text.append("disk.usableBytes=unknown\n");
                }
            }
            ArcpTuning.writeFile(statusDirectory, "status.txt", text.toString());
        } catch (RuntimeException failure) {
            report(failure);
        }
    }

    private void report(RuntimeException failure) {
        String message = String.valueOf(failure.getMessage());
        if (!message.equals(lastFailure)) {
            lastFailure = message;
            DriverStation.reportWarning("Athena ARCP: " + message, false);
        }
    }

    private static long micros(double seconds) {
        return Double.isFinite(seconds) ? (long) (seconds * 1_000_000.0) : 0L;
    }

    static Path defaultDirectory(boolean simulation, String robotPath, String simulationName) {
        return simulation ? Path.of(System.getProperty("java.io.tmpdir"), simulationName) : Path.of("/home/lvuser", robotPath);
    }

    static Path defaultDeployedTuning(boolean simulation) {
        return simulation ? Path.of("src/main/deploy/athena/tuning") : Path.of("/home/lvuser/deploy/athena/tuning");
    }

    @Override
    public void close() {
        if (closed) {
            return;
        }
        closed = true;
        if (server != null) {
            server.close();
        }
    }
}
