package ca.frc6390.athena.core.loop;

import java.util.Objects;
import java.util.function.Consumer;

/**
 * Generic runner metadata/state for fixed-rate loop and hook invocations.
 */
public final class TimedRunner<T> {
    private final String name;
    private final T task;
    private final double periodSeconds;
    private double lastRunSeconds = Double.NaN;
    private double lastOutput;

    public TimedRunner(String name, double periodSeconds, T task) {
        this.name = name;
        this.task = Objects.requireNonNull(task, "task");
        this.periodSeconds = Math.max(0.0, periodSeconds);
    }

    public static <T> TimedRunner<T> periodicMs(T task, double periodMs) {
        return new TimedRunner<>(null, periodMs / 1000.0, task);
    }

    public String name() {
        return name;
    }

    public T task() {
        return task;
    }

    public double periodSeconds() {
        return periodSeconds;
    }

    public double periodMs() {
        return periodSeconds * 1000.0;
    }

    public double lastRunSeconds() {
        return lastRunSeconds;
    }

    public void setLastRunSeconds(double nowSeconds) {
        this.lastRunSeconds = nowSeconds;
    }

    public double lastOutput() {
        return lastOutput;
    }

    public void setLastOutput(double output) {
        this.lastOutput = output;
    }

    public boolean shouldRunSeconds(double nowSeconds) {
        if (!Double.isFinite(nowSeconds)) {
            return true;
        }
        if (Double.isNaN(lastRunSeconds)) {
            return true;
        }
        if (periodSeconds <= 0.0) {
            return true;
        }
        return (nowSeconds - lastRunSeconds) >= periodSeconds;
    }

    public void run(Consumer<T> invoker, double nowSeconds) {
        invoker.accept(task);
        lastRunSeconds = nowSeconds;
    }

    public void reset() {
        lastRunSeconds = Double.NaN;
    }
}
