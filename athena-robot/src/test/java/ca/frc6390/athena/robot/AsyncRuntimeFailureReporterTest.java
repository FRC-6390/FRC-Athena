package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.*;
import java.time.Duration;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class AsyncRuntimeFailureReporterTest {
    @Test void heldDestinationDoesNotDelayRobotAndOverloadIsCounted() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        CountDownLatch drained = new CountDownLatch(3);
        var executor = new java.util.concurrent.ThreadPoolExecutor(1, 1, 0, TimeUnit.SECONDS,
                new java.util.concurrent.ArrayBlockingQueue<Runnable>(2));
        AsyncRuntimeFailureReporter reporter = new AsyncRuntimeFailureReporter(new RuntimeFailureReporter() {
            @Override public void error(String message, Throwable cause) {
                entered.countDown();
                try { release.await(); }
                catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); }
                drained.countDown();
            }
            @Override public void warning(String message, Throwable cause) { error(message, cause); }
        }, executor);
        try {
            reporter.error("first", null);
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            assertTimeout(Duration.ofMillis(100), () -> {
                for (int i = 0; i < 200; i++) reporter.warning("pending " + i, null);
            });
            assertEquals(198, reporter.droppedReports());
            assertTrue(reporter.lastFailure().contains("queue full"));
        } finally { release.countDown(); executor.shutdown(); }
        assertTrue(drained.await(2, TimeUnit.SECONDS));
    }
}
