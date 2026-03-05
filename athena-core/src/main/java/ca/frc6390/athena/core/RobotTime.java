package ca.frc6390.athena.core;

/**
 * Caches the current FPGA timestamp once per robot loop so telemetry/shuffleboard polling
 * doesn't hammer HAL timing calls (which becomes very expensive with many widgets).
 *
 * <p>Updated from {@link LoopTiming#beginCycle()}.</p>
 */
public final class RobotTime {
    private RobotTime() {}

    private static final double MAX_PROJECTED_AGE_SECONDS = 0.25;

    public record LoopSnapshot(double nowSeconds, double loopDtSeconds, long cycleCount) {
        public boolean hasNowSeconds() {
            return Double.isFinite(nowSeconds);
        }

        public boolean hasLoopDtSeconds() {
            return Double.isFinite(loopDtSeconds) && loopDtSeconds >= 0.0;
        }
    }

    private static volatile double nowSeconds = Double.NaN;
    private static volatile double loopDtSeconds = Double.NaN;
    private static volatile long loopCycleCount = 0L;
    private static volatile long lastUpdateNanoTime = 0L;
    private static volatile LoopSnapshot loopSnapshot =
            new LoopSnapshot(Double.NaN, Double.NaN, 0L);

    static synchronized void updateNowSeconds(double nowSeconds) {
        double dtSeconds = Double.NaN;
        if (Double.isFinite(nowSeconds) && Double.isFinite(RobotTime.nowSeconds)) {
            double candidate = nowSeconds - RobotTime.nowSeconds;
            if (Double.isFinite(candidate) && candidate >= 0.0) {
                dtSeconds = candidate;
            }
        }
        RobotTime.nowSeconds = nowSeconds;
        RobotTime.loopDtSeconds = dtSeconds;
        RobotTime.loopCycleCount = RobotTime.loopCycleCount + 1L;
        RobotTime.lastUpdateNanoTime = System.nanoTime();
        RobotTime.loopSnapshot = new LoopSnapshot(nowSeconds, dtSeconds, RobotTime.loopCycleCount);
    }

    /**
     * Test hook for clearing the cached loop timestamp.
     */
    public static synchronized void resetNowSecondsForTest() {
        RobotTime.nowSeconds = Double.NaN;
        RobotTime.loopDtSeconds = Double.NaN;
        RobotTime.loopCycleCount = 0L;
        RobotTime.lastUpdateNanoTime = 0L;
        RobotTime.loopSnapshot = new LoopSnapshot(Double.NaN, Double.NaN, 0L);
    }

    public static double nowSeconds() {
        return nowSeconds;
    }

    /**
     * Returns a projected timestamp using the last FPGA sample plus elapsed JVM monotonic time.
     *
     * <p>This keeps high-rate loops from observing a completely stale value between base loop
     * updates, while still avoiding repeated HAL timestamp reads.</p>
     */
    public static double nowSecondsProjected() {
        double now = nowSeconds;
        if (!Double.isFinite(now)) {
            return Double.NaN;
        }
        long updatedAt = lastUpdateNanoTime;
        if (updatedAt <= 0L) {
            return now;
        }
        double elapsedSeconds = (System.nanoTime() - updatedAt) * 1e-9;
        if (!Double.isFinite(elapsedSeconds) || elapsedSeconds <= 0.0) {
            return now;
        }
        return now + Math.min(elapsedSeconds, MAX_PROJECTED_AGE_SECONDS);
    }

    /**
     * Returns cached now unless the cached sample is older than {@code staleAfterSeconds}, then
     * returns a monotonic-time projection.
     */
    public static double nowSecondsProjectedIfStale(double staleAfterSeconds) {
        double now = nowSeconds;
        if (!Double.isFinite(now)) {
            return Double.NaN;
        }
        long updatedAt = lastUpdateNanoTime;
        if (updatedAt <= 0L) {
            return now;
        }
        double ageSeconds = (System.nanoTime() - updatedAt) * 1e-9;
        if (!Double.isFinite(ageSeconds) || ageSeconds <= Math.max(0.0, staleAfterSeconds)) {
            return now;
        }
        return now + Math.min(ageSeconds, MAX_PROJECTED_AGE_SECONDS);
    }

    public static double loopDtSeconds() {
        return loopDtSeconds;
    }

    public static long loopCycleCount() {
        return loopCycleCount;
    }

    public static LoopSnapshot loopSnapshot() {
        return loopSnapshot;
    }
}
