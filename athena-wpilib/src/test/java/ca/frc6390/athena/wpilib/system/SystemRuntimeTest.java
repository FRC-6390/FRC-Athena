package ca.frc6390.athena.wpilib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class SystemRuntimeTest {
    private static final long MIB = 1024L * 1024L;

    @Test
    void automaticTunesOnlyRealLowMemoryLinuxTargets() {
        FakeAccess lowMemory = new FakeAccess(256 * MIB, 80 * MIB);
        SystemRuntime runtime = new SystemRuntime(SystemTuning.automatic(), lowMemory);
        runtime.applyPlan();

        assertTrue(runtime.status().tuningComplete());
        assertEquals(List.of("web", "vm.overcommit_memory=1", "swap=32", "vm.swappiness=20"), lowMemory.calls);
        assertEquals(4, runtime.status().appliedChanges().size());

        FakeAccess rioTwo = new FakeAccess(2_048 * MIB, 1_000 * MIB);
        SystemRuntime rioTwoRuntime = new SystemRuntime(SystemTuning.automatic(), rioTwo);
        rioTwoRuntime.applyPlan();
        assertTrue(rioTwoRuntime.status().tuningComplete());
        assertTrue(rioTwo.calls.isEmpty());

        FakeAccess unknown = new FakeAccess(256 * MIB, 80 * MIB);
        unknown.target = "Unknown Linux robot";
        new SystemRuntime(SystemTuning.automatic(), unknown).applyPlan();
        assertTrue(unknown.calls.isEmpty());
    }

    @Test
    void standardIsMonitorOnlyAndExplicitLowMemoryCanUseExistingZram() {
        FakeAccess standard = new FakeAccess(256 * MIB, 80 * MIB);
        new SystemRuntime(SystemTuning.standard(), standard).applyPlan();
        assertTrue(standard.calls.isEmpty());

        FakeAccess zram = new FakeAccess(2_048 * MIB, 400 * MIB);
        zram.zram = true;
        SystemRuntime explicit = new SystemRuntime(SystemTuning.lowMemory(), zram);
        explicit.applyPlan();
        assertFalse(zram.calls.stream().anyMatch(call -> call.startsWith("swap=")));
        assertTrue(zram.calls.contains("vm.swappiness=100"));
        assertTrue(explicit.status().appliedChanges().contains("Using active zram swap"));
    }

    @Test
    void classifiesPressureAndKeepsTuningFailuresInStatus() {
        FakeAccess access = new FakeAccess(256 * MIB, 10 * MIB);
        access.failSysctl = true;
        SystemRuntime runtime = new SystemRuntime(SystemTuning.automatic(), access);
        assertEquals(MemoryPressure.CRITICAL, runtime.status().pressure());

        runtime.applyPlan();
        assertEquals(2, runtime.status().failures().size());
        assertTrue(runtime.status().failures().get(0).contains("permission denied"));

        access.memory = new SystemAccess.Memory(256 * MIB, 20 * MIB, 70 * MIB, 0, 0);
        SystemRuntime warning = new SystemRuntime(SystemTuning.automatic(), access);
        assertEquals(MemoryPressure.WARNING, warning.status().pressure());
    }

    private static final class FakeAccess implements SystemAccess {
        private final List<String> calls = new ArrayList<>();
        private Memory memory;
        private boolean zram;
        private boolean failSysctl;
        private String target = "Test roboRIO";

        private FakeAccess(long total, long available) {
            memory = new Memory(total, available, 70 * MIB, 0, 0);
        }

        @Override public boolean realRobot() { return true; }
        @Override public boolean linux() { return true; }
        @Override public String target() { return target; }
        @Override public Memory memory() { return memory; }
        @Override public boolean hasActiveZram() { return zram; }

        @Override
        public Result stopNiWebServer() {
            calls.add("web");
            return Result.success("stopped");
        }

        @Override
        public Result setSysctl(String key, int value) {
            calls.add(key + "=" + value);
            return failSysctl ? Result.failure("permission denied") : Result.success("set");
        }

        @Override
        public Result enableSwapFile(long bytes) {
            calls.add("swap=" + bytes / MIB);
            return Result.success("enabled");
        }
    }
}
