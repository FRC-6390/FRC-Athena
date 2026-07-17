package ca.frc6390.athena.wpilib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class MemoryPressureMonitorTest {
    private static final long MIB = 1024L * 1024L;

    @Test
    void hysteresisPreventsThresholdFlappingAndRequiresRecovery() {
        SystemTuning tuning = SystemTuning.automatic().pressureHysteresis(3, 4);
        MemoryPressureMonitor monitor = new MemoryPressureMonitor(tuning);
        SystemAccess.Memory warning = memory(30);
        SystemAccess.Memory normal = memory(100);

        assertEquals(MemoryPressure.NORMAL, monitor.update(warning, 0, -1, 0).pressure());
        assertEquals(MemoryPressure.NORMAL, monitor.update(normal, 0, -1, 0).pressure());
        monitor.update(warning, 0, -1, 0);
        monitor.update(warning, 0, -1, 0);
        assertEquals(MemoryPressure.WARNING, monitor.update(warning, 0, -1, 0).pressure());

        for (int index = 0; index < 3; index++) {
            assertEquals(MemoryPressure.WARNING, monitor.update(normal, 0, -1, 0).pressure());
        }
        assertEquals(MemoryPressure.NORMAL, monitor.update(normal, 0, -1, 0).pressure());
    }

    @Test
    void hardFloorEscalatesImmediatelyAndTrendCanWarnBeforeThreshold() {
        MemoryPressureMonitor hardFloor = new MemoryPressureMonitor(SystemTuning.automatic());
        assertEquals(MemoryPressure.CRITICAL, hardFloor.update(memory(7), 0, -1, 0).pressure());

        MemoryPressureMonitor trend = new MemoryPressureMonitor(
                SystemTuning.automatic().pressureHysteresis(2, 4));
        trend.update(memory(100), -5 * MIB, 20, 0);
        assertEquals(MemoryPressure.WARNING, trend.update(memory(95), -5 * MIB, 19, 0).pressure());
    }

    @Test
    void criticalPressureCanRecoverToWarningAfterSustainedImprovement() {
        MemoryPressureMonitor monitor = new MemoryPressureMonitor(
                SystemTuning.automatic().pressureHysteresis(2, 3));
        assertEquals(MemoryPressure.CRITICAL, monitor.update(memory(7), 0, -1, 0).pressure());
        assertEquals(MemoryPressure.CRITICAL, monitor.update(memory(30), 0, -1, 0).pressure());
        assertEquals(MemoryPressure.CRITICAL, monitor.update(memory(30), 0, -1, 0).pressure());
        assertEquals(MemoryPressure.WARNING, monitor.update(memory(30), 0, -1, 0).pressure());
    }

    private static SystemAccess.Memory memory(long availableMib) {
        return new SystemAccess.Memory(256 * MIB, availableMib * MIB, 70 * MIB, 64 * MIB, 0);
    }
}
