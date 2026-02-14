package ca.frc6390.athena.core.loop;

import java.util.Objects;
import java.util.function.Consumer;

/**
 * Generic runner metadata/state for fixed-rate periodic hooks.
 */
public final class TimedPeriodicHookRunner<H> {
    private final H hook;
    private final double periodMs;
    private double lastRunMs = Double.NaN;

    public TimedPeriodicHookRunner(H hook, double periodMs) {
        this.hook = Objects.requireNonNull(hook, "hook");
        this.periodMs = Math.max(0.0, periodMs);
    }

    public H hook() {
        return hook;
    }

    public double periodMs() {
        return periodMs;
    }

    public boolean shouldRun(double nowMs) {
        if (!Double.isFinite(nowMs)) {
            return true;
        }
        if (Double.isNaN(lastRunMs)) {
            return true;
        }
        if (periodMs <= 0.0) {
            return true;
        }
        return (nowMs - lastRunMs) >= periodMs;
    }

    public void run(Consumer<H> invoker, double nowMs) {
        invoker.accept(hook);
        lastRunMs = nowMs;
    }

    public void reset() {
        lastRunMs = Double.NaN;
    }
}
