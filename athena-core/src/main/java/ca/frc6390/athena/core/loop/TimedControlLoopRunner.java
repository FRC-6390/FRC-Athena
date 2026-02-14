package ca.frc6390.athena.core.loop;

/**
 * Generic runner metadata/state for fixed-rate control loops.
 */
public final class TimedControlLoopRunner<L> {
    private final String name;
    private final double periodSeconds;
    private final L loop;
    private double lastRunSeconds = Double.NaN;
    private double lastOutput;

    public TimedControlLoopRunner(String name, double periodSeconds, L loop) {
        this.name = name;
        this.periodSeconds = Math.max(0.0, periodSeconds);
        this.loop = loop;
    }

    public String name() {
        return name;
    }

    public double periodSeconds() {
        return periodSeconds;
    }

    public L loop() {
        return loop;
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

    public boolean shouldRun(double nowSeconds) {
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
}
