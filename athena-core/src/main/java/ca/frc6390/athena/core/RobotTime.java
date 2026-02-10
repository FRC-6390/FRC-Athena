package ca.frc6390.athena.core;

/**
 * Caches the current FPGA timestamp once per robot loop so telemetry/shuffleboard polling
 * doesn't hammer HAL timing calls (which becomes very expensive with many widgets).
 *
 * <p>Updated from {@link LoopTiming#beginCycle()}.</p>
 */
final class RobotTime {
    private RobotTime() {}

    private static volatile double nowSeconds = Double.NaN;

    static void updateNowSeconds(double nowSeconds) {
        RobotTime.nowSeconds = nowSeconds;
    }

    static double nowSeconds() {
        return nowSeconds;
    }
}

