package ca.frc6390.athena.runtime.control;

import java.util.Objects;
import java.util.function.Consumer;

/**
 * Deterministic periodic runner for small runtime tasks.
 *
 * @param <T> task type
 */
public final class TimedRunner<T> {
    private final T task;
    private final double periodSeconds;
    private double lastRunSeconds = Double.NaN;

    /**
     * Creates a runner.
     *
     * @param task task instance
     * @param periodSeconds minimum period in seconds
     */
    public TimedRunner(T task, double periodSeconds) {
        this.task = Objects.requireNonNull(task, "task");
        this.periodSeconds = Double.isFinite(periodSeconds) ? periodSeconds : 0.0;
    }

    /**
     * Creates a millisecond-periodic runner.
     *
     * @param task task instance
     * @param periodMs minimum period in milliseconds
     * @return timed runner
     * @param <T> task type
     */
    public static <T> TimedRunner<T> periodicMs(T task, double periodMs) {
        return new TimedRunner<>(task, periodMs / 1000.0);
    }

    /**
     * Returns whether a run is due at the supplied timestamp.
     *
     * @param nowSeconds current timestamp in seconds
     * @return true when due
     */
    public boolean shouldRunSeconds(double nowSeconds) {
        if (!Double.isFinite(nowSeconds) || !Double.isFinite(lastRunSeconds)) {
            return true;
        }
        return periodSeconds <= 0.0 || nowSeconds - lastRunSeconds >= periodSeconds;
    }

    /**
     * Runs the task when due.
     *
     * @param runner task runner
     * @param nowSeconds current timestamp in seconds
     * @return true when the task ran
     */
    public boolean run(Consumer<T> runner, double nowSeconds) {
        Objects.requireNonNull(runner, "runner");
        if (!shouldRunSeconds(nowSeconds)) {
            return false;
        }
        runner.accept(task);
        lastRunSeconds = nowSeconds;
        return true;
    }

    /**
     * Clears scheduling history so the next check is due.
     */
    public void reset() {
        lastRunSeconds = Double.NaN;
    }
}
