package ca.frc6390.athena.arcp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.Duration;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class CyclePublisherTest {
    @Test
    void saturatedQueueCoalescesLatestDeltasWithoutWaitingAndKeepsOrder() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        List<Long> timestamps = new ArrayList<>();
        Map<Long, Object> latest = new LinkedHashMap<>();
        CyclePublisher publisher = new CyclePublisher((handle, timestamp, bytes, length) -> {
            entered.countDown();
            try {
                assertTrue(release.await(3, TimeUnit.SECONDS), "test writer was never released");
            } catch (InterruptedException failure) {
                throw new IllegalStateException(failure);
            }
            timestamps.add(timestamp);
            ArcpMarshal.In in = new ArcpMarshal.In(bytes);
            int count = in.u32();
            for (int i = 0; i < count; i++) latest.put(in.u64(), in.value());
        });
        try {
            publish(publisher, 0, new ArcpCycle().setNumber(1, 0));
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            for (int i = 1; i <= CyclePublisher.CAPACITY; i++) {
                publish(publisher, i, new ArcpCycle().setNumber(100 + i, i).setNumber(1, i));
            }
            long before = System.nanoTime();
            publish(publisher, 9, new ArcpCycle().setNumber(109, 9).setNumber(1, 9));
            publish(publisher, 10, new ArcpCycle().setNumber(110, 10));
            assertTrue(Duration.ofNanos(System.nanoTime() - before).toMillis() < 100,
                    "a full queue must not wait for the blocked publisher");
            release.countDown();
            publisher.awaitIdle();
            assertEquals(List.of(0L, 1L, 2L, 3L, 4L, 5L, 6L, 7L, 10L), timestamps);
            assertEquals(9.0, latest.get(1L));
            for (int i = 1; i <= 10; i++) assertEquals((double) i, latest.get(100L + i));
        } finally {
            release.countDown();
            assertTrue(publisher.close());
        }
    }

    @Test
    void queuedCycleOwnsItsArrayValues() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        List<Object> received = new ArrayList<>();
        CyclePublisher publisher = new CyclePublisher((handle, timestamp, bytes, length) -> {
            entered.countDown();
            try {
                release.await(3, TimeUnit.SECONDS);
            } catch (InterruptedException failure) {
                throw new IllegalStateException(failure);
            }
            ArcpMarshal.In in = new ArcpMarshal.In(bytes);
            assertEquals(1, in.u32());
            in.u64();
            received.add(in.value());
        });
        try {
            double[] values = {1.0, 2.0};
            publish(publisher, 0, new ArcpCycle().set(1, values));
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            values[0] = 99;
            release.countDown();
            publisher.awaitIdle();
            assertEquals(1.0, ((double[]) received.get(0))[0]);
            assertFalse(received.isEmpty());
        } finally {
            release.countDown();
            assertTrue(publisher.close());
        }
    }

    @Test
    void blockedPublisherDoesNotTurnSchemaMarkersOrRecordsIntoRobotBarriers() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        List<String> order = new ArrayList<>();
        CyclePublisher publisher = new CyclePublisher((handle, timestamp, bytes, length) -> {
            if (timestamp == 0) {
                entered.countDown();
                try { assertTrue(release.await(5, TimeUnit.SECONDS)); }
                catch (InterruptedException interrupted) { throw new IllegalStateException(interrupted); }
            }
            order.add("cycle" + timestamp);
        });
        try {
            publish(publisher, 0, new ArcpCycle().setNumber(1, 0));
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            long before = System.nanoTime();
            publisher.execute(() -> order.add("dictionary"), 100);
            publish(publisher, 1, new ArcpCycle().setNumber(2, 1));
            publisher.execute(() -> order.add("marker"), 100);
            publisher.execute(() -> order.add("records"), 100);
            publish(publisher, 2, new ArcpCycle().setNumber(2, 2));
            assertTrue(System.nanoTime() - before < TimeUnit.MILLISECONDS.toNanos(100));
            release.countDown();
            publisher.awaitIdle();
            assertEquals(List.of("cycle0", "dictionary", "cycle1", "marker", "records", "cycle2"), order);
        } finally { release.countDown(); assertTrue(publisher.close()); }
    }

    @Test
    void criticalOverloadRejectsExplicitlyAndStopsNewPublicationWithoutWaiting() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        CyclePublisher publisher = new CyclePublisher((handle, timestamp, bytes, length) -> {
            entered.countDown();
            try { release.await(5, TimeUnit.SECONDS); }
            catch (InterruptedException interrupted) { throw new IllegalStateException(interrupted); }
        });
        try {
            publish(publisher, 0, new ArcpCycle());
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            for (int i = 0; i < CyclePublisher.CAPACITY - 1; i++) publisher.execute(() -> { }, 1);
            org.junit.jupiter.api.Assertions.assertThrows(IllegalStateException.class,
                    () -> publisher.execute(() -> { }, 1));
            assertFalse(publisher.failureMessage().isEmpty());
            org.junit.jupiter.api.Assertions.assertThrows(IllegalStateException.class,
                    () -> publish(publisher, 1, new ArcpCycle()));
        } finally { release.countDown(); assertTrue(publisher.close()); }
    }

    @Test
    void oversizedPublicationIsRejectedBeforeReadingOrAllocatingItsPayload() {
        CyclePublisher publisher = new CyclePublisher((handle, timestamp, bytes, length) -> { });
        try {
            org.junit.jupiter.api.Assertions.assertThrows(IllegalStateException.class,
                    () -> publisher.publish(1, 0, java.nio.ByteBuffer.allocate(0), CyclePublisher.MAX_BYTES + 1));
            assertTrue(publisher.failureMessage().contains("byte budget"));
        } finally { assertTrue(publisher.close()); }
    }

    @Test
    void linkageFailureFailsClosedAndShutdownStillDrains() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        CyclePublisher publisher = new CyclePublisher((handle, timestamp, bytes, length) -> {
            entered.countDown();
            try { release.await(5, TimeUnit.SECONDS); }
            catch (InterruptedException interrupted) { throw new IllegalStateException(interrupted); }
            throw new UnsatisfiedLinkError("native missing");
        });
        publish(publisher, 0, new ArcpCycle());
        assertTrue(entered.await(1, TimeUnit.SECONDS));
        publisher.execute(() -> { throw new AssertionError("cannot continue after failed predecessor"); }, 1);
        release.countDown();
        assertTrue(publisher.close());
        assertTrue(publisher.failureMessage().contains("discarded operations=1"));
    }

    private static void publish(CyclePublisher publisher, long timestamp, ArcpCycle cycle) {
        ArcpMarshal.Out out = new ArcpMarshal.Out();
        out.cycle(cycle);
        publisher.publish(1L, timestamp, out.buffer(), out.length());
    }
}
