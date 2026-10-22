package ca.frc6390.athena.wpilib.arcp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import java.io.IOException;
import java.time.Duration;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class DiskSpaceMonitorTest {
    @Test
    void blockedMountQueryLeavesRequestsCachedStatusAndCloseResponsive() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        CountDownLatch finished = new CountDownLatch(1);
        AtomicInteger probes = new AtomicInteger();
        DiskSpaceMonitor monitor = new DiskSpaceMonitor(() -> {
            probes.incrementAndGet();
            entered.countDown();
            try {
                release.await();
                return 123L;
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                throw new IOException(exception);
            } finally {
                finished.countDown();
            }
        });
        try {
            monitor.request();
            assertTrue(entered.await(2, TimeUnit.SECONDS));
            assertTimeoutPreemptively(Duration.ofMillis(200), () -> {
                for (int cycle = 0; cycle < 1_000; cycle++) {
                    monitor.request();
                    assertEquals(-1L, monitor.usableBytes());
                }
                monitor.close();
                monitor.request();
            });
            assertEquals(1, probes.get(), "requests coalesce while disk is blocked");
            assertTrue(finished.await(2, TimeUnit.SECONDS));
            assertEquals(-1L, monitor.usableBytes(), "closed monitor never publishes a late result");
        } finally {
            release.countDown();
            monitor.close();
        }
    }

    @Test
    void unavailableDiskCanRecoverOnTheNextBackgroundRequest() {
        AtomicInteger probes = new AtomicInteger();
        try (DiskSpaceMonitor monitor = new DiskSpaceMonitor(() -> {
            if (probes.incrementAndGet() == 1) throw new IOException("mount unavailable");
            return 987L;
        })) {
            monitor.request();
            assertTimeoutPreemptively(Duration.ofSeconds(2), () -> {
                while (monitor.usableBytes() != 987L) {
                    monitor.request();
                    Thread.yield();
                }
            });
            assertEquals(987L, monitor.usableBytes());
        }
    }
}
