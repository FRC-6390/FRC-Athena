package ca.frc6390.athena.wpilib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;

class LinuxSystemAccessTest {
    private static final long KIB = 1024L;

    @Test
    void estimatesAvailableMemoryOnOlderRoborioKernels() {
        SystemAccess.Memory memory = LinuxSystemAccess.parseMemory(List.of(
                "MemTotal:        262144 kB",
                "MemFree:          12000 kB",
                "Buffers:           2000 kB",
                "Cached:           18000 kB",
                "SReclaimable:      3000 kB",
                "Shmem:             1000 kB",
                "SwapTotal:        65536 kB",
                "SwapFree:         49152 kB"), List.of(
                "Name:\tjava",
                "VmRSS:\t75000 kB"));

        assertEquals(262144 * KIB, memory.total());
        assertEquals(34000 * KIB, memory.available());
        assertEquals(75000 * KIB, memory.rss());
        assertEquals(65536 * KIB, memory.swapTotal());
        assertEquals(16384 * KIB, memory.swapUsed());
    }

    @Test
    void prefersKernelMemAvailableWhenPresent() {
        SystemAccess.Memory memory = LinuxSystemAccess.parseMemory(List.of(
                "MemTotal:        262144 kB",
                "MemAvailable:     42000 kB",
                "MemFree:          12000 kB",
                "Cached:           18000 kB",
                "SwapTotal:            0 kB",
                "SwapFree:             0 kB"), List.of());

        assertEquals(42000 * KIB, memory.available());
    }
}
