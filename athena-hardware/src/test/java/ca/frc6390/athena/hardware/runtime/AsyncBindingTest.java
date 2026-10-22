package ca.frc6390.athena.hardware.runtime;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class AsyncBindingTest {
    @Test
    void firstSynchronousSetupPrepareDoesNotOwnTheGraphMonitor() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        MotorDevice blocked = MotorDevice.of(MotorKinds.KRAKEN_X60, 80);
        MotorDevice healthy = MotorDevice.of(MotorKinds.KRAKEN_X60, 81);
        var caller = Executors.newSingleThreadExecutor();
        try (HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend(blocked, entered, release)))) {
            var first = caller.submit(() -> graph.motor(blocked));
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            MotorHandle second = graph.motor(healthy);
            assertSame(second, graph.motor(healthy));
            assertFalse(first.isDone());
            release.countDown();
            assertTrue(first.get(1, TimeUnit.SECONDS).configurationStatus().ready());
        } finally {
            release.countDown();
            caller.shutdownNow();
        }
    }

    @Test
    void backgroundFirstBindReturnsPendingAndHealthyBindingsProceedIndependently() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        MotorDevice blocked = MotorDevice.of(MotorKinds.KRAKEN_X60, 82);
        MotorDevice healthy = MotorDevice.of(MotorKinds.KRAKEN_X60, 83);
        try (HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend(blocked, entered, release)))
                .asynchronousBindings()) {
            assertFalse(graph.motor(blocked).configurationStatus().ready());
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            assertTrue(graph.bindingFailures().isEmpty(), "ordinary pending binding is not a device failure");
            graph.motor(healthy);
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(1);
            while (!graph.motor(healthy).configurationStatus().ready() && System.nanoTime() < deadline) Thread.sleep(1);
            assertTrue(graph.motor(healthy).configurationStatus().ready());
            assertFalse(graph.motor(blocked).configurationStatus().ready());
            release.countDown();
            assertTrue(graph.awaitBindingsIdle(1000));
            assertTrue(graph.motor(blocked).configurationStatus().ready());
        } finally { release.countDown(); }
    }

    private static MotorBackend backend(MotorDevice blocked, CountDownLatch entered, CountDownLatch release) {
        return new MotorBackend() {
            @Override public boolean supports(MotorKind kind) { return true; }
            @Override public MotorHandle create(MotorDevice device) {
                if (device.id() == blocked.id()) {
                    entered.countDown();
                    try { release.await(); }
                    catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
                }
                return new MotorHandle() { @Override public MotorDevice device() { return device; } };
            }
        };
    }

    @Test
    void equalPhysicalDeclarationsCannotPrepareConcurrentRecoveryHandles() throws Exception {
        var attempts = new java.util.concurrent.atomic.AtomicInteger();
        var clock = new java.util.concurrent.atomic.AtomicLong();
        var entered = new CountDownLatch(1);
        var release = new CountDownLatch(1);
        MotorBackend backend = new MotorBackend() {
            @Override public boolean supports(MotorKind kind) { return true; }
            @Override public MotorHandle create(MotorDevice device) {
                int attempt = attempts.incrementAndGet();
                if (attempt <= 2) throw new IllegalStateException("temporary constructor failure");
                entered.countDown();
                try { release.await(); }
                catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
                return new MotorHandle() { @Override public MotorDevice device() { return device; } };
            }
        };
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 86);
        MotorDevice alias = MotorDevice.of(MotorKinds.KRAKEN_X60, 86);
        try (HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend))) {
            graph.setBindingClock(clock::get);
            graph.motor(first);
            graph.motor(alias);
            assertEquals(2, attempts.get());
            clock.set(TimeUnit.MILLISECONDS.toNanos(101));
            graph.refreshInputs();
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            graph.refreshInputs();
            assertEquals(3, attempts.get(), "the alias must wait while the physical identity is preparing");
            release.countDown();
            assertTrue(graph.awaitBindingsIdle(1000));
            graph.refreshInputs();
            assertTrue(graph.awaitBindingsIdle(1000));
            assertSame(graph.motor(first), graph.motor(alias));
            assertEquals(3, attempts.get());
            assertTrue(graph.bindingFailures().isEmpty());
        } finally { release.countDown(); }
    }

    @Test
    void closeRemovesCancelledQueuedDescriptorsAndDisposesLateActiveHandles() throws Exception {
        var entered = new CountDownLatch(4);
        var release = new CountDownLatch(1);
        var creates = new java.util.concurrent.atomic.AtomicInteger();
        var closes = new java.util.concurrent.atomic.AtomicInteger();
        MotorBackend backend = new MotorBackend() {
            @Override public boolean supports(MotorKind kind) { return true; }
            @Override public MotorHandle create(MotorDevice device) {
                creates.incrementAndGet();
                entered.countDown();
                boolean done = false;
                while (!done) {
                    try { release.await(); done = true; }
                    catch (InterruptedException ignored) { /* emulate held JNI construction */ }
                }
                return new ClosingMotor(device, closes);
            }
        };
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend)).asynchronousBindings();
        try {
            for (int id = 100; id < 107; id++) graph.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, id));
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            graph.close();
            assertFalse(graph.awaitBindingsIdle(5), "active native prepares are still accounted for");
            release.countDown();
            assertTrue(graph.awaitBindingsIdle(1000), "cancelled queued descriptors cannot remain stranded");
            assertEquals(4, creates.get());
            assertEquals(4, closes.get());
        } finally { release.countDown(); graph.close(); }
    }

    private record ClosingMotor(MotorDevice device, java.util.concurrent.atomic.AtomicInteger closes)
            implements MotorHandle, AutoCloseable {
        @Override public void close() { closes.incrementAndGet(); }
    }
}
