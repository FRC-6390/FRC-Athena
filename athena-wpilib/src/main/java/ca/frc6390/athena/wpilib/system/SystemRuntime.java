package ca.frc6390.athena.wpilib.system;

import java.lang.management.BufferPoolMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryUsage;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicReference;

/** Owns low-frequency system monitoring and one-time roboRIO tuning. */
public final class SystemRuntime implements AutoCloseable {
    private static final long LOW_MEMORY_TARGET_MAX_BYTES = 512L * 1024L * 1024L;
    private static final long SAMPLE_PERIOD_NANOS = 1_000_000_000L;

    private final SystemTuning tuning;
    private final SystemAccess access;
    private final List<String> applied = new CopyOnWriteArrayList<>();
    private final List<String> failures = new CopyOnWriteArrayList<>();
    private final AtomicReference<SystemStatus> status;
    private volatile boolean tuningComplete;
    private volatile boolean closed;
    private long nextSampleNanos;

    public static SystemRuntime create(SystemTuning tuning, boolean realRobot) {
        return new SystemRuntime(tuning, new LinuxSystemAccess(realRobot));
    }

    SystemRuntime(SystemTuning tuning, SystemAccess access) {
        this.tuning = tuning == null ? SystemTuning.automatic() : tuning;
        this.access = access;
        tuningComplete = this.tuning.profile() == SystemTuning.Profile.DISABLED;
        status = new AtomicReference<>(sample());
    }

    /** Starts the one-time tuning plan without delaying robot initialization. */
    public void start() {
        if (tuning.profile() == SystemTuning.Profile.DISABLED) return;
        Thread worker = new Thread(this::applyPlan, "Athena-SystemTuning");
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
            SystemAccess.Memory memory = access.memory();
            String target = access.target().toLowerCase(java.util.Locale.ROOT);
            boolean automaticLowMemory = tuning.profile() == SystemTuning.Profile.AUTOMATIC
                    && access.realRobot() && access.linux()
                    && target.contains("roborio")
                    && memory.total() > 0 && memory.total() <= LOW_MEMORY_TARGET_MAX_BYTES;
            boolean explicitLowMemory = tuning.profile() == SystemTuning.Profile.LOW_MEMORY
                    && access.realRobot() && access.linux();
            if (!automaticLowMemory && !explicitLowMemory) return;

            if (tuning.stopNiWebServer()) apply("WebServer", access.stopNiWebServer());
            apply("Overcommit", access.setSysctl("vm.overcommit_memory", 1));
            boolean zram = access.hasActiveZram();
            if (!zram && memory.swapTotal() <= 0) {
                apply("Swap", access.enableSwapFile(tuning.swapBytes()));
            } else {
                applied.add(zram ? "Using active zram swap" : "Using existing swap");
            }
            apply("Swappiness", access.setSysctl("vm.swappiness", zram ? 100 : 20));
        } finally {
            tuningComplete = true;
            status.set(sample());
        }
    }

    private void apply(String operation, SystemAccess.Result result) {
        if (result.success()) applied.add(operation + ": " + result.detail());
        else failures.add(operation + ": " + result.detail());
    }

    private SystemStatus sample() {
        SystemAccess.Memory memory = access.memory();
        MemoryUsage heap = ManagementFactory.getMemoryMXBean().getHeapMemoryUsage();
        long direct = ManagementFactory.getPlatformMXBeans(BufferPoolMXBean.class).stream()
                .filter(pool -> pool.getName().equalsIgnoreCase("direct"))
                .mapToLong(BufferPoolMXBean::getMemoryUsed)
                .sum();
        return new SystemStatus(
                tuning.profile(), access.target(), pressure(memory), memory.total(), memory.available(), memory.rss(),
                heap.getUsed(), heap.getMax(), direct, memory.swapTotal(), memory.swapUsed(), tuningComplete,
                applied, failures);
    }

    private MemoryPressure pressure(SystemAccess.Memory memory) {
        if (memory.total() <= 0 || memory.available() < 0) return MemoryPressure.NORMAL;
        double available = (double) memory.available() / memory.total();
        if (available <= tuning.criticalAvailableFraction()) return MemoryPressure.CRITICAL;
        if (available <= tuning.warningAvailableFraction()) return MemoryPressure.WARNING;
        return MemoryPressure.NORMAL;
    }

    @Override
    public void close() {
        closed = true;
    }
}
