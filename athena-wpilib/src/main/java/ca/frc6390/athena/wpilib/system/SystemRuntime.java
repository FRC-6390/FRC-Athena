package ca.frc6390.athena.wpilib.system;

import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/** Owns low-frequency system monitoring and one-time roboRIO tuning. */
public final class SystemRuntime implements AutoCloseable {
    private static final long LOW_MEMORY_TARGET_MAX_BYTES = 512L * 1024L * 1024L;
    private static final long SAMPLE_PERIOD_NANOS = 1_000_000_000L;
    private static final double TREND_ALPHA = 0.35;

    private final SystemTuning tuning;
    private final SystemAccess access;
    private final SystemTuningStateStore stateStore;
    private final MemoryPressureMonitor pressureMonitor;
    private final List<String> applied = new CopyOnWriteArrayList<>();
    private final List<String> failures = new CopyOnWriteArrayList<>();
    private final AtomicReference<SystemStatus> status;
    private final AtomicBoolean started = new AtomicBoolean();
    private volatile boolean tuningComplete;
    private volatile boolean closed;
    private volatile Thread worker;
    private long nextSampleNanos;
    private long previousSampleNanos;
    private SystemAccess.Memory previousMemory;
    private JvmMetrics previousJvm;
    private double availableTrend;

    public static SystemRuntime create(SystemTuning tuning, boolean realRobot) {
        return new SystemRuntime(tuning, new LinuxSystemAccess(realRobot), SystemTuningStateStore.rio());
    }

    SystemRuntime(SystemTuning tuning, SystemAccess access) {
        this(tuning, access, new MemoryStateStore());
    }

    SystemRuntime(SystemTuning tuning, SystemAccess access, SystemTuningStateStore stateStore) {
        this.tuning = tuning == null ? SystemTuning.automatic() : tuning;
        this.access = access;
        this.stateStore = stateStore;
        pressureMonitor = new MemoryPressureMonitor(this.tuning);
        tuningComplete = this.tuning.profile() == SystemTuning.Profile.DISABLED;
        status = new AtomicReference<>(sample());
        nextSampleNanos = System.nanoTime() + SAMPLE_PERIOD_NANOS;
    }

    /** Starts the one-time tuning or restoration plan without delaying robot initialization. */
    public void start() {
        if (tuning.profile() == SystemTuning.Profile.DISABLED || !started.compareAndSet(false, true)) return;
        worker = new Thread(this::applyPlan, "Athena-SystemTuning");
        worker.setDaemon(true);
        worker.setPriority(Thread.MIN_PRIORITY);
        worker.start();
    }

    /** Refreshes the status snapshot at most once per second. */
    public void update() {
        if (closed || tuning.profile() == SystemTuning.Profile.DISABLED) return;
        long now = System.nanoTime();
        if (now < nextSampleNanos) return;
        nextSampleNanos = now + SAMPLE_PERIOD_NANOS;
        status.set(sample());
    }

    public SystemStatus status() {
        return status.get();
    }

    void applyPlan() {
        try {
            if (tuning.profile() == SystemTuning.Profile.RESTORE_DEFAULTS) {
                if (!access.realRobot() || !access.linux()) {
                    applied.add("Restore skipped because this is not a real Linux robot target");
                    return;
                }
                restoreDefaults();
                return;
            }
            SystemAccess.Memory memory = access.memory();
            String target = access.target().toLowerCase(Locale.ROOT);
            boolean automaticLowMemory = tuning.profile() == SystemTuning.Profile.AUTOMATIC
                    && access.realRobot() && access.linux() && target.contains("roborio")
                    && memory.total() > 0 && memory.total() <= LOW_MEMORY_TARGET_MAX_BYTES;
            boolean explicitLowMemory = tuning.profile() == SystemTuning.Profile.LOW_MEMORY
                    && access.realRobot() && access.linux();
            if (!automaticLowMemory && !explicitLowMemory) return;

            SystemTuningState state = stateStore.load().orElseGet(() -> new SystemTuningState(
                    access.readSysctl("vm.overcommit_memory"),
                    access.readSysctl("vm.swappiness"),
                    access.niWebServerRunning(),
                    false,
                    false));
            if (!integer(state.overcommitMemory()) || !integer(state.swappiness())) {
                failures.add("State: original sysctl values could not be captured; no OS changes were made");
                return;
            }
            if (!persist(state)) return;

            if (tuning.stopNiWebServer() && state.webServerRunning()) {
                applyRequired("WebServer", access.stopNiWebServer());
            }
            boolean zram = access.hasActiveZram();
            boolean fallbackSwap = access.hasActiveSwapFile();
            if (!zram && access.zramSupported()) {
                SystemTuningState ownership = state.withZram(true);
                if (!persist(ownership)) return;
                SystemAccess.Result result = access.enableZram(tuning.zramBytes());
                if (result.success()) {
                    applied.add("Zram: " + result.detail());
                    zram = true;
                    state = ownership;
                } else {
                    applied.add("Zram unavailable; trying bounded file swap: " + result.detail());
                    persist(state);
                }
            }
            if (!zram && memory.swapTotal() <= 0 && !access.hasActiveSwapFile()) {
                SystemTuningState ownership = state.withSwapFile(true);
                if (!persist(ownership)) return;
                SystemAccess.Result result = access.enableSwapFile(
                        tuning.swapBytes(), tuning.minimumFreeDiskBytes());
                if (result.success()) {
                    applied.add("Swap: " + result.detail());
                    state = ownership;
                    fallbackSwap = true;
                } else {
                    failures.add("Swap: " + result.detail());
                    persist(state);
                }
            } else if (zram) {
                applied.add("Using verified zram swap");
            } else {
                applied.add("Using existing swap");
            }
            boolean memoryBacking = zram || fallbackSwap || memory.swapTotal() > 0;
            if (memoryBacking) {
                applyRequired("Overcommit", access.setSysctl("vm.overcommit_memory", 1));
                applyRequired("Swappiness", access.setSysctl("vm.swappiness", zram ? 100 : 20));
            } else {
                failures.add("Overcommit: not enabled because no swap backing could be verified");
            }
        } finally {
            tuningComplete = true;
            status.set(sample());
        }
    }

    private void restoreDefaults() {
        Optional<SystemTuningState> stored = stateStore.load();
        if (stored.isEmpty()) {
            applied.add("No Athena tuning state exists; nothing to restore");
            return;
        }
        SystemTuningState state = stored.get();
        boolean restored = true;
        if (state.athenaZram()) restored &= applyRequired("Zram restore", access.disableZram());
        if (state.athenaSwapFile()) restored &= applyRequired("Swap restore", access.disableSwapFile());
        restored &= restoreSysctl("vm.overcommit_memory", state.overcommitMemory());
        restored &= restoreSysctl("vm.swappiness", state.swappiness());
        if (state.webServerRunning()) restored &= applyRequired("WebServer restore", access.startNiWebServer());
        if (restored) applyRequired("State restore", stateStore.delete());
    }

    private boolean restoreSysctl(String key, String value) {
        try {
            return applyRequired(key + " restore", access.setSysctl(key, Integer.parseInt(value)));
        } catch (NumberFormatException exception) {
            failures.add(key + " restore: captured value is invalid: '" + value + "'");
            return false;
        }
    }

    private boolean persist(SystemTuningState state) {
        return applyRequired("State", stateStore.save(state));
    }

    private boolean applyRequired(String operation, SystemAccess.Result result) {
        if (result.success()) {
            applied.add(operation + ": " + result.detail());
            return true;
        }
        failures.add(operation + ": " + result.detail());
        return false;
    }

    private synchronized SystemStatus sample() {
        long now = System.nanoTime();
        SystemAccess.Memory memory = access.memory();
        JvmMetrics jvm = JvmMetrics.capture();
        double elapsedSeconds = previousSampleNanos == 0 ? 0.0 : (now - previousSampleNanos) / 1.0e9;
        boolean stableInterval = elapsedSeconds >= 0.5;
        double allocationRate = stableInterval
                ? rate(jvm.allocatedBytes(), previousJvm == null ? -1 : previousJvm.allocatedBytes(), elapsedSeconds)
                : -1.0;
        long nativeMemory = estimatedNative(memory, jvm);
        long previousNativeMemory = previousMemory == null || previousJvm == null
                ? -1 : estimatedNative(previousMemory, previousJvm);
        double directGrowth = stableInterval && previousJvm != null
                ? deltaRate(jvm.directBuffers(), previousJvm.directBuffers(), elapsedSeconds) : 0.0;
        double nativeGrowth = stableInterval
                ? deltaRate(nativeMemory, previousNativeMemory, elapsedSeconds) : 0.0;
        double residentGrowth = stableInterval && previousMemory != null
                ? deltaRate(memory.rss(), previousMemory.rss(), elapsedSeconds) : 0.0;
        double rawTrend = stableInterval && previousMemory != null
                && memory.available() >= 0 && previousMemory.available() >= 0
                ? (memory.available() - previousMemory.available()) / elapsedSeconds
                : 0.0;
        availableTrend = previousSampleNanos == 0 || !stableInterval
                ? rawTrend
                : TREND_ALPHA * rawTrend + (1.0 - TREND_ALPHA) * availableTrend;
        double gcLoad = previousJvm == null || !stableInterval
                ? 0.0
                : Math.max(0.0, jvm.garbageCollectionMillis() - previousJvm.garbageCollectionMillis())
                        / (elapsedSeconds * 1000.0);
        double secondsToExhaustion = availableTrend < -256.0 * 1024.0 && memory.available() >= 0
                ? memory.available() / -availableTrend
                : -1.0;
        MemoryPressureMonitor.Result pressure = pressureMonitor.update(
                memory, availableTrend, secondsToExhaustion, gcLoad);
        JvmRecommendations.Recommendation recommendation = JvmRecommendations.forTarget(
                access.realRobot(), access.target(), memory.total(), jvm.heapMaximum());
        previousSampleNanos = now;
        previousMemory = memory;
        previousJvm = jvm;
        return new SystemStatus(
                tuning.profile(), access.target(), pressure.pressure(), pressure.reason(),
                memory.total(), memory.available(), pressure.lowestAvailableBytes(), memory.rss(),
                jvm.heapUsed(), jvm.heapCommitted(), jvm.heapMaximum(), jvm.nonHeapUsed(), jvm.directBuffers(),
                directGrowth, nativeMemory, nativeGrowth, residentGrowth, allocationRate,
                pressure.availableTrendBytesPerSecond(), pressure.estimatedSecondsToExhaustion(),
                jvm.garbageCollections(), jvm.garbageCollectionMillis(), gcLoad,
                jvm.processCpuLoad(), jvm.systemCpuLoad(), jvm.liveThreads(),
                memory.swapTotal(), memory.swapUsed(), access.swapKind(), pressure.transitions(),
                tuningComplete, tuningComplete && failures.isEmpty(), recommendation.healthy(),
                recommendation.arguments(), applied, failures);
    }

    private static double rate(long current, long previous, double elapsedSeconds) {
        if (current < 0 || previous < 0 || elapsedSeconds <= 0.0) return -1.0;
        return Math.max(0.0, current - previous) / elapsedSeconds;
    }

    private static boolean integer(String value) {
        try {
            Integer.parseInt(value);
            return true;
        } catch (NumberFormatException ignored) {
            return false;
        }
    }

    private static double deltaRate(long current, long previous, double elapsedSeconds) {
        if (current < 0 || previous < 0 || elapsedSeconds <= 0.0) return 0.0;
        return (current - previous) / elapsedSeconds;
    }

    private static long estimatedNative(SystemAccess.Memory memory, JvmMetrics jvm) {
        if (memory.rss() < 0) return -1;
        return Math.max(0L, memory.rss() - Math.max(0L, jvm.heapUsed())
                - Math.max(0L, jvm.nonHeapUsed()) - Math.max(0L, jvm.directBuffers()));
    }

    @Override
    public void close() {
        closed = true;
        Thread active = worker;
        if (active != null) active.interrupt();
    }

    private static final class MemoryStateStore implements SystemTuningStateStore {
        private SystemTuningState state;
        @Override public Optional<SystemTuningState> load() { return Optional.ofNullable(state); }
        @Override public SystemAccess.Result save(SystemTuningState state) {
            this.state = state;
            return SystemAccess.Result.success("state saved");
        }
        @Override public SystemAccess.Result delete() {
            state = null;
            return SystemAccess.Result.success("state removed");
        }
    }
}
