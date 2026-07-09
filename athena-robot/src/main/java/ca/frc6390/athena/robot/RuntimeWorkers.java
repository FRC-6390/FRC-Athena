package ca.frc6390.athena.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

/**
 * Configurable runtime worker group.
 */
public final class RuntimeWorkers implements AutoCloseable {
    private final List<RuntimeWorker> workers;
    private final ScheduledExecutorService executor;
    private final double[] lastRunSeconds;
    private final List<ScheduledFuture<?>> futures = new ArrayList<>();
    private final List<Failure> failures = new ArrayList<>();
    private Consumer<Failure> failureHandler = failure -> {
    };
    private boolean started;

    private RuntimeWorkers(List<RuntimeWorker> workers, ScheduledExecutorService executor) {
        this.workers = List.copyOf(workers == null ? List.of() : workers);
        this.executor = executor;
        this.lastRunSeconds = new double[this.workers.size()];
        for (int i = 0; i < lastRunSeconds.length; i++) {
            lastRunSeconds[i] = Double.NaN;
        }
    }

    /**
     * Creates an empty worker group.
     *
     * @return worker group
     */
    public static RuntimeWorkers none() {
        return new RuntimeWorkers(List.of(), null);
    }

    /**
     * Creates workers run from the main runtime loop when due.
     *
     * @param workers workers
     * @return worker group
     */
    public static RuntimeWorkers inline(RuntimeWorker... workers) {
        return new RuntimeWorkers(workers == null ? List.of() : List.of(workers), null);
    }

    /**
     * Creates workers run by a caller-supplied scheduler.
     *
     * @param executor executor
     * @param workers workers
     * @return worker group
     */
    public static RuntimeWorkers async(ScheduledExecutorService executor, RuntimeWorker... workers) {
        return new RuntimeWorkers(
                workers == null ? List.of() : List.of(workers),
                Objects.requireNonNull(executor, "executor"));
    }

    /**
     * Returns configured workers.
     *
     * @return workers
     */
    public List<RuntimeWorker> workers() {
        return workers;
    }

    /**
     * Configures a failure sink. Worker failures are still recorded even when no sink is configured.
     *
     * @param failureHandler failure sink
     * @return this worker group
     */
    public RuntimeWorkers onFailure(Consumer<Failure> failureHandler) {
        this.failureHandler = failureHandler == null ? failure -> {
        } : failureHandler;
        return this;
    }

    /**
     * Returns recorded worker failures.
     *
     * @return failures
     */
    public synchronized List<Failure> failures() {
        return List.copyOf(failures);
    }

    /**
     * Starts async workers. Inline workers are run by {@link #runDue(double)}.
     *
     * @return this worker group
     */
    public synchronized RuntimeWorkers start() {
        if (started || executor == null) {
            started = true;
            return this;
        }
        for (RuntimeWorker worker : workers) {
            long periodNanos = Math.max(1L, Math.round(worker.periodSeconds() * 1_000_000_000.0));
            futures.add(executor.scheduleAtFixedRate(
                    () -> runWorker(worker),
                    0L,
                    periodNanos,
                    TimeUnit.NANOSECONDS));
        }
        started = true;
        return this;
    }

    /**
     * Runs due inline workers from the main runtime loop.
     *
     * @param nowSeconds runtime timestamp
     */
    public void runDue(double nowSeconds) {
        if (executor != null || !Double.isFinite(nowSeconds)) {
            return;
        }
        for (int i = 0; i < workers.size(); i++) {
            RuntimeWorker worker = workers.get(i);
            double lastRun = lastRunSeconds[i];
            if (Double.isNaN(lastRun) || nowSeconds - lastRun >= worker.periodSeconds()) {
                lastRunSeconds[i] = nowSeconds;
                runWorker(worker);
            }
        }
    }

    /**
     * Cancels async workers. The caller-owned executor is not shut down.
     */
    @Override
    public synchronized void close() {
        for (ScheduledFuture<?> future : futures) {
            future.cancel(false);
        }
        futures.clear();
        started = false;
    }

    private void runWorker(RuntimeWorker worker) {
        try {
            worker.task().run();
        } catch (RuntimeException exception) {
            recordFailure(new Failure(worker, exception));
        }
    }

    private void recordFailure(Failure failure) {
        synchronized (this) {
            failures.add(failure);
        }
        failureHandler.accept(failure);
    }

    /**
     * A worker failure recorded without stopping future worker runs.
     *
     * @param worker worker that failed
     * @param exception thrown exception
     */
    public record Failure(RuntimeWorker worker, RuntimeException exception) {
        public Failure {
            Objects.requireNonNull(worker, "worker");
            Objects.requireNonNull(exception, "exception");
        }
    }
}
