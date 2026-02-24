package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.loop.TimedRunner;

/**
 * Example motion-limit aggregation and timed-runner helpers.
 */
public final class MotionLimitsExamples {
    private MotionLimitsExamples() {}

    public static MotionLimits createDriveLimitProfile(RobotSpeeds speeds) {
        MotionLimits limits = new MotionLimits();
        limits.setBaseDriveLimits(MotionLimits.DriveLimits.fromRobotSpeeds(speeds));
        limits.registerDriveProvider(() -> new MotionLimits.DriveLimits(3.2, 4.0, 1.9, 6.0));
        limits.registerDriveProvider(() -> new MotionLimits.DriveLimits(Double.NaN, 2.5, 0.0, 5.0));
        return limits;
    }

    public static MotionLimits createAxisLimitProfile() {
        MotionLimits limits = new MotionLimits();
        limits.setBaseAxisLimits("arm", new MotionLimits.AxisLimits(2.0, 3.0));
        limits.registerAxisProvider("arm", () -> new MotionLimits.AxisLimits(1.5, 4.0));
        limits.registerAxisProvider("arm", () -> new MotionLimits.AxisLimits(Double.NaN, 2.0));
        return limits;
    }

    public static TimedRunner<Runnable> every20Ms(Runnable task) {
        return TimedRunner.periodicMs(task, 20.0);
    }
}
