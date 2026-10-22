package ca.frc6390.athena.robot;

import java.util.Objects;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

/** Bounded reporting to an I/O destination; the robot never waits for stderr or Driver Station. */
public final class AsyncRuntimeFailureReporter implements RuntimeFailureReporter {
    private static final ThreadPoolExecutor WORKER = worker();
    private final RuntimeFailureReporter destination;
    private final java.util.concurrent.Executor executor;
    private final AtomicLong dropped = new AtomicLong();
    private volatile String lastFailure = "";

    private static ThreadPoolExecutor worker() {
        ThreadPoolExecutor executor = new ThreadPoolExecutor(1, 1, 10, TimeUnit.SECONDS,
                new ArrayBlockingQueue<>(128), task -> {
                    Thread thread = new Thread(task, "athena-failure-reporter");
                    thread.setDaemon(true);
                    thread.setPriority(Thread.MIN_PRIORITY);
                    return thread;
                });
        executor.allowCoreThreadTimeOut(true);
        return executor;
    }

    AsyncRuntimeFailureReporter(RuntimeFailureReporter destination) { this(destination, WORKER); }

    AsyncRuntimeFailureReporter(RuntimeFailureReporter destination, java.util.concurrent.Executor executor) {
        this.executor = Objects.requireNonNull(executor, "executor");
        this.destination = Objects.requireNonNull(destination, "destination");
    }

    @Override public void error(String message, Throwable cause) { submit(true, message, cause); }
    @Override public void warning(String message, Throwable cause) { submit(false, message, cause); }
    public long droppedReports() { return dropped.get(); }
    public String lastFailure() { return lastFailure; }

    private void submit(boolean error, String message, Throwable cause) {
        try {
            executor.execute(() -> {
                try {
                    if (error) destination.error(message, cause);
                    else destination.warning(message, cause);
                } catch (Throwable failure) { lastFailure = failure.toString(); }
            });
        } catch (java.util.concurrent.RejectedExecutionException full) {
            dropped.incrementAndGet();
            lastFailure = "Failure reporting queue full";
        }
    }
}
