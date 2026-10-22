package ca.frc6390.athena.hardware.backend;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class ConfigurationQueueTest {
    @Test
    void coalescesSupersededValuesPreservesOrderAndDoesNotAcknowledgeAStaleCompletion() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        CountDownLatch latestEntered = new CountDownLatch(1);
        CountDownLatch latestRelease = new CountDownLatch(1);
        List<String> writes = new ArrayList<>();
        try (ConfigurationQueue queue = new ConfigurationQueue("blocked", () -> { });
                ConfigurationQueue healthy = new ConfigurationQueue("healthy", () -> { })) {
            queue.request(ConfigurationQueue.Key.RUNTIME, "first", () -> {
                entered.countDown();
                await(release);
                writes.add("first");
            });
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            queue.request(ConfigurationQueue.Key.RUNTIME, "superseded", () -> writes.add("superseded"));
            queue.request(ConfigurationQueue.Key.RUNTIME, "latest", () -> {
                latestEntered.countDown();
                await(latestRelease);
                writes.add("latest");
            });
            queue.request(ConfigurationQueue.Key.POSITION, 0.0, () -> writes.add("zero"));
            healthy.request(ConfigurationQueue.Key.RUNTIME, "ready", () -> { });
            assertTrue(healthy.awaitReady(1000), "one blocked controller does not hold healthy configuration");
            release.countDown();
            assertTrue(latestEntered.await(1, TimeUnit.SECONDS));
            assertEquals(4, queue.status().requestedGeneration());
            assertEquals(0, queue.status().appliedGeneration(), "old completion cannot acknowledge newer work");
            assertFalse(queue.status().ready());
            latestRelease.countDown();
            assertTrue(queue.awaitReady(1000));
            assertEquals(List.of("first", "latest", "zero"), writes);
            assertEquals(4, queue.status().appliedGeneration());
        } finally {
            release.countDown();
            latestRelease.countDown();
        }
    }

    @Test
    void failedConfigurationIsCachedAndAChangedRequestCanRecover() throws Exception {
        try (ConfigurationQueue queue = new ConfigurationQueue("failure", () -> { })) {
            queue.request(ConfigurationQueue.Key.RUNTIME, "bad", () -> { throw new IllegalStateException("CAN failed"); });
            assertFalse(queue.awaitReady(1000));
            assertTrue(queue.status().failed());
            assertTrue(queue.status().error().contains("CAN failed"));
            queue.request(ConfigurationQueue.Key.RUNTIME, "good", () -> { });
            assertTrue(queue.awaitReady(1000));
            assertEquals(2, queue.status().appliedGeneration());
        }
    }

    @Test
    void workerSaturationRetainsEveryRejectedConfigurationKindForRetry() throws Exception {
        var entered = new CountDownLatch(1);
        var release = new CountDownLatch(1);
        var executor = new java.util.concurrent.ThreadPoolExecutor(1, 1, 1, TimeUnit.SECONDS,
                new java.util.concurrent.ArrayBlockingQueue<Runnable>(1));
        executor.execute(() -> { entered.countDown(); await(release); });
        assertTrue(entered.await(1, TimeUnit.SECONDS));
        executor.execute(() -> { }); // fill the ready queue while the only worker is held
        List<String> writes = new ArrayList<>();
        try (ConfigurationQueue queue = new ConfigurationQueue("saturated", () -> { }, executor)) {
            queue.request(ConfigurationQueue.Key.POSITION, 0.0, () -> writes.add("zero"));
            queue.request(ConfigurationQueue.Key.RUNTIME, "runtime", () -> writes.add("runtime"));
            assertTrue(queue.status().failed());
            release.countDown();
            Thread.sleep(1100); // the rejection backoff is one second
            queue.retryFailed();
            assertTrue(queue.awaitReady(1000), "both rejected configuration kinds must recover");
            assertEquals(List.of("zero", "runtime"), writes, "retry retains submission order across kinds");
        } finally {
            release.countDown();
            executor.shutdownNow();
        }
    }

    @Test
    void failedRequiredConfigurationIsRetriedBeforeLaterZeroingCanRun() throws Exception {
        AtomicInteger attempts = new AtomicInteger();
        List<String> writes = new ArrayList<>();
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        try (ConfigurationQueue queue = new ConfigurationQueue("ordered failure", () -> { })) {
            queue.request(ConfigurationQueue.Key.RUNTIME, "runtime", () -> {
                if (attempts.incrementAndGet() == 1) {
                    entered.countDown();
                    await(release);
                    throw new IllegalStateException("temporary CAN failure");
                }
                writes.add("runtime");
            });
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            queue.request(ConfigurationQueue.Key.POSITION, 0.0, () -> writes.add("zero"));
            release.countDown();
            assertFalse(queue.awaitReady(1000));
            assertTrue(writes.isEmpty(), "zeroing cannot overtake failed required configuration");
            Thread.sleep(110);
            queue.retryFailed();
            assertTrue(queue.awaitReady(1000));
            assertEquals(List.of("runtime", "zero"), writes);
        } finally { release.countDown(); }
    }

    @Test
    void closeReturnsWhileDriverIsBlockedAndFencesItsLateCompletionBeforeNativeCleanup() throws Exception {
        ConfigurationQueue queue = new ConfigurationQueue("close", () -> { });
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        CountDownLatch cleaned = new CountDownLatch(1);
        AtomicInteger cleanupCalls = new AtomicInteger();
        queue.request(ConfigurationQueue.Key.RUNTIME, "blocked", () -> {
            entered.countDown();
            await(release);
        });
        assertTrue(entered.await(1, TimeUnit.SECONDS));
        long start = System.nanoTime();
        queue.close(() -> { cleanupCalls.incrementAndGet(); cleaned.countDown(); });
        assertTrue(System.nanoTime() - start < TimeUnit.MILLISECONDS.toNanos(100));
        assertEquals(0, cleanupCalls.get(), "native object stays alive until the in-flight call exits");
        assertFalse(queue.status().ready());
        release.countDown();
        assertTrue(cleaned.await(1, TimeUnit.SECONDS));
        assertEquals(1, cleanupCalls.get());
        assertFalse(queue.status().ready(), "late success cannot resurrect a closed controller");
        assertEquals("Closed", queue.status().error());
    }

    private static void await(CountDownLatch latch) {
        boolean complete = false;
        while (!complete) {
            try { latch.await(); complete = true; }
            catch (InterruptedException ignored) { /* emulate a native driver that ignores cancellation */ }
        }
    }
}
