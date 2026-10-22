package ca.frc6390.athena.wpilib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.time.Duration;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class SystemRuntimeTest {
    private static final long MIB = 1024L * 1024L;

    @Test
    void automaticTunesOnlyIdentifiedLowMemoryRoborio() {
        FakeAccess lowMemory = new FakeAccess(256 * MIB, 80 * MIB);
        SystemRuntime runtime = new SystemRuntime(SystemTuning.automatic(), lowMemory);
        runtime.applyPlan();

        assertTrue(runtime.status().tuningComplete());
        assertTrue(runtime.status().tuningVerified());
        assertEquals(List.of("webStop", "vm.overcommit_memory=1", "vm.vfs_cache_pressure=1000",
                        "swap=32", "vm.swappiness=20"),
                lowMemory.calls);

        FakeAccess rioTwo = new FakeAccess(2_048 * MIB, 1_000 * MIB);
        new SystemRuntime(SystemTuning.automatic(), rioTwo).applyPlan();
        assertTrue(rioTwo.calls.isEmpty());

        FakeAccess unknown = new FakeAccess(256 * MIB, 80 * MIB);
        unknown.target = "Unknown Linux robot";
        new SystemRuntime(SystemTuning.automatic(), unknown).applyPlan();
        assertTrue(unknown.calls.isEmpty());
    }

    @Test
    void prefersZramAndFallsBackWithoutTreatingUnavailableZramAsFatal() {
        FakeAccess zram = new FakeAccess(256 * MIB, 80 * MIB);
        zram.zramSupported = true;
        SystemRuntime zramRuntime = new SystemRuntime(SystemTuning.automatic(), zram);
        zramRuntime.applyPlan();
        assertTrue(zram.calls.contains("zram=64"));
        assertFalse(zram.calls.stream().anyMatch(call -> call.startsWith("swap=")));
        assertTrue(zram.calls.contains("vm.swappiness=100"));

        FakeAccess fallback = new FakeAccess(256 * MIB, 80 * MIB);
        fallback.zramSupported = true;
        fallback.failZram = true;
        SystemRuntime fallbackRuntime = new SystemRuntime(SystemTuning.automatic(), fallback);
        fallbackRuntime.applyPlan();
        assertTrue(fallback.calls.contains("swap=32"));
        assertTrue(fallbackRuntime.status().tuningVerified());
        assertTrue(fallbackRuntime.status().appliedChanges().stream()
                .anyMatch(change -> change.contains("bounded file swap")));
    }

    @Test
    void restoreUsesPersistedOriginalStateAndRemovesAthenaResources() {
        FakeAccess access = new FakeAccess(256 * MIB, 80 * MIB);
        FakeStore store = new FakeStore(new SystemTuningState("0", "100", "60", true, true, true));
        SystemRuntime runtime = new SystemRuntime(SystemTuning.restoreDefaults(), access, store);
        runtime.applyPlan();

        assertEquals(List.of("zramOff", "swapOff", "vm.overcommit_memory=0",
                        "vm.vfs_cache_pressure=100", "vm.swappiness=60", "webStart"),
                access.calls);
        assertTrue(store.deleted);
        assertTrue(runtime.status().tuningVerified());
    }

    @Test
    void failureIsNonfatalAndVisibleInStatus() {
        FakeAccess access = new FakeAccess(256 * MIB, 7 * MIB);
        access.failSysctl = true;
        SystemRuntime runtime = new SystemRuntime(SystemTuning.automatic(), access);
        runtime.applyPlan();
        assertEquals(MemoryPressure.CRITICAL, runtime.status().pressure());
        assertFalse(runtime.status().tuningVerified());
        assertEquals(3, runtime.status().failures().size());
        assertTrue(runtime.status().failures().get(0).contains("permission denied"));
    }

    @Test
    void enablesOvercommitEvenWhenSwapCannotBeVerified() {
        FakeAccess access = new FakeAccess(256 * MIB, 40 * MIB);
        access.failSwap = true;
        SystemRuntime runtime = new SystemRuntime(SystemTuning.automatic(), access);
        runtime.applyPlan();

        assertTrue(access.calls.contains("vm.overcommit_memory=1"));
        assertTrue(runtime.status().failures().stream()
                .anyMatch(failure -> failure.startsWith("Swap:")));
    }

    @Test
    void explicitLowMemoryProfileUsesMoreHeadroomThanAutomatic() {
        SystemTuning automatic = SystemTuning.automatic();
        SystemTuning lowMemory = SystemTuning.lowMemory();

        assertTrue(lowMemory.zramBytes() > automatic.zramBytes());
        assertTrue(lowMemory.swapBytes() > automatic.swapBytes());
        assertTrue(lowMemory.warningAvailableFraction() > automatic.warningAvailableFraction());
        assertTrue(lowMemory.criticalAvailableFraction() > automatic.criticalAvailableFraction());
        assertTrue(lowMemory.escalationSamples() < automatic.escalationSamples());
    }

    @Test
    void constructorUpdateStatusAndCloseNeverWaitForBlockedSystemMetrics() throws Exception {
        FakeAccess access = new FakeAccess(256 * MIB, 80 * MIB);
        access.metricEntered = new CountDownLatch(1);
        access.metricRelease = new CountDownLatch(1);
        SystemRuntime runtime = new SystemRuntime(SystemTuning.automatic(), access);
        try {
            assertEquals(0, access.metricCalls.get(), "constructor performs no OS reads");
            assertEquals("Unknown", runtime.status().target());
            assertEquals(-1L, runtime.status().availableMemoryBytes());
            runtime.start();
            assertTrue(access.metricEntered.await(2, TimeUnit.SECONDS));
            assertTimeoutPreemptively(Duration.ofMillis(200), () -> {
                for (int cycle = 0; cycle < 1_000; cycle++) {
                    runtime.update();
                    assertEquals(-1L, runtime.status().availableMemoryBytes());
                }
                runtime.close();
            });
            assertEquals(1, access.metricCalls.get(), "one monitor worker owns the blocked call");
            assertTrue(access.metricThread.startsWith("Athena-SystemMonitor"));
        } finally {
            access.metricRelease.countDown();
            runtime.close();
        }
    }

    @Test
    void workerPublishesTuningOnceAndKeepsEarlierSnapshotsImmutable() throws Exception {
        FakeAccess access = new FakeAccess(256 * MIB, 80 * MIB);
        try (SystemRuntime runtime = new SystemRuntime(SystemTuning.automatic(), access)) {
            SystemStatus pending = runtime.status();
            runtime.start();
            for (int cycle = 0; cycle < 100; cycle++) runtime.update();
            assertTimeoutPreemptively(Duration.ofSeconds(2), () -> {
                while (!runtime.status().tuningComplete()) Thread.yield();
            });
            assertTrue(runtime.status().tuningVerified());
            assertEquals(1, access.calls.stream().filter("vm.overcommit_memory=1"::equals).count());
            assertEquals(80 * MIB, runtime.status().availableMemoryBytes());
            assertFalse(pending.tuningComplete());
            assertEquals(-1L, pending.availableMemoryBytes());
        }
    }

    private static final class FakeAccess implements SystemAccess {
        private final List<String> calls = new ArrayList<>();
        private Memory memory;
        private boolean zram;
        private boolean zramSupported;
        private boolean failZram;
        private boolean failSysctl;
        private boolean failSwap;
        private String target = "Test roboRIO";
        private CountDownLatch metricEntered;
        private CountDownLatch metricRelease;
        private final AtomicInteger metricCalls = new AtomicInteger();
        private volatile String metricThread = "";

        private FakeAccess(long total, long available) {
            memory = new Memory(total, available, 70 * MIB, 0, 0);
        }

        @Override public boolean realRobot() { return true; }
        @Override public boolean linux() { return true; }
        @Override public String target() { return target; }
        @Override public Memory memory() {
            metricCalls.incrementAndGet();
            metricThread = Thread.currentThread().getName();
            if (metricEntered != null) {
                metricEntered.countDown();
                try {
                    metricRelease.await();
                } catch (InterruptedException exception) {
                    Thread.currentThread().interrupt();
                }
            }
            return memory;
        }
        @Override public String swapKind() { return zram ? "ZRAM" : "NONE"; }
        @Override public String readSysctl(String key) {
            if (key.endsWith("swappiness")) return "60";
            if (key.endsWith("vfs_cache_pressure")) return "100";
            return "0";
        }
        @Override public boolean niWebServerRunning() { return true; }
        @Override public boolean zramSupported() { return zramSupported; }
        @Override public boolean hasActiveZram() { return zram; }
        @Override public boolean hasActiveSwapFile() { return false; }
        @Override public long usableSwapDiskBytes() { return 500 * MIB; }

        @Override public Result stopNiWebServer() { calls.add("webStop"); return Result.success("stopped"); }
        @Override public Result startNiWebServer() { calls.add("webStart"); return Result.success("started"); }

        @Override
        public Result setSysctl(String key, int value) {
            calls.add(key + "=" + value);
            return failSysctl ? Result.failure("permission denied") : Result.success("set");
        }

        @Override
        public Result enableZram(long bytes) {
            calls.add("zram=" + bytes / MIB);
            if (failZram) return Result.failure("module missing");
            zram = true;
            return Result.success("enabled");
        }

        @Override public Result disableZram() { calls.add("zramOff"); zram = false; return Result.success("off"); }

        @Override
        public Result enableSwapFile(long bytes, long minimumFreeDiskBytes) {
            calls.add("swap=" + bytes / MIB);
            return failSwap ? Result.failure("disk reserve") : Result.success("enabled");
        }

        @Override public Result disableSwapFile() { calls.add("swapOff"); return Result.success("off"); }
    }

    private static final class FakeStore implements SystemTuningStateStore {
        private SystemTuningState state;
        private boolean deleted;
        private FakeStore(SystemTuningState state) { this.state = state; }
        @Override public Optional<SystemTuningState> load() { return Optional.ofNullable(state); }
        @Override public SystemAccess.Result save(SystemTuningState state) {
            this.state = state;
            return SystemAccess.Result.success("saved");
        }
        @Override public SystemAccess.Result delete() {
            state = null;
            deleted = true;
            return SystemAccess.Result.success("deleted");
        }
    }
}
