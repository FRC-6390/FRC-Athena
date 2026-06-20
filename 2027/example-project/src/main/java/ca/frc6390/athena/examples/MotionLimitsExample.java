package ca.frc6390.athena.examples;

import ca.frc6390.athena.runtime.control.MotionLimits;
import ca.frc6390.athena.runtime.control.TimedRunner;

/**
 * Motion limit aggregation and timed runner examples.
 */
public final class MotionLimitsExample {
    private MotionLimitsExample() {
    }

    /**
     * Creates a drive limit profile with conservative runtime providers.
     *
     * @return configured motion limits
     */
    public static MotionLimits driveLimitProfile() {
        return new MotionLimits()
                .baseDriveLimits(new MotionLimits.DriveLimits(4.5, 3.0, 2.2, 7.0))
                .driveProvider(() -> new MotionLimits.DriveLimits(3.2, 4.0, 1.9, 6.0))
                .driveProvider(() -> new MotionLimits.DriveLimits(Double.NaN, 2.5, 0.0, 5.0));
    }

    /**
     * Creates an arm axis limit profile with conservative runtime providers.
     *
     * @return configured motion limits
     */
    public static MotionLimits axisLimitProfile() {
        return new MotionLimits()
                .baseAxisLimits("arm", new MotionLimits.AxisLimits(2.0, 3.0))
                .axisProvider("arm", () -> new MotionLimits.AxisLimits(1.5, 4.0))
                .axisProvider("arm", () -> new MotionLimits.AxisLimits(Double.NaN, 2.0));
    }

    /**
     * Creates a 20 ms periodic runner.
     *
     * @param task task to run
     * @return timed runner
     */
    public static TimedRunner<Runnable> every20Ms(Runnable task) {
        return TimedRunner.periodicMs(task, 20.0);
    }
}
