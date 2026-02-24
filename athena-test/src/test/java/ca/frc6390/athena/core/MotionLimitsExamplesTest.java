package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.MotionLimitsExamples;
import ca.frc6390.athena.core.loop.TimedRunner;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

final class MotionLimitsExamplesTest {

    @Test
    void driveLimitsResolveToMostConservativeFiniteValues() {
        RobotSpeeds speeds = new RobotSpeeds(4.5, 3.0);
        MotionLimits limits = MotionLimitsExamples.createDriveLimitProfile(speeds);

        MotionLimits.DriveLimits resolved = limits.resolveDriveLimits();
        assertEquals(3.2, resolved.maxLinearVelocity(), 1e-9);
        assertEquals(2.5, resolved.maxLinearAcceleration(), 1e-9);
        assertEquals(1.9, resolved.maxAngularVelocity(), 1e-9);
        assertEquals(5.0, resolved.maxAngularAcceleration(), 1e-9);
    }

    @Test
    void axisLimitsResolveAndUnknownAxisReturnsNone() {
        MotionLimits limits = MotionLimitsExamples.createAxisLimitProfile();

        MotionLimits.AxisLimits arm = limits.resolveAxisLimits("arm");
        assertEquals(1.5, arm.maxVelocity(), 1e-9);
        assertEquals(2.0, arm.maxAcceleration(), 1e-9);

        MotionLimits.AxisLimits unknown = limits.resolveAxisLimits("wrist");
        assertEquals(0.0, unknown.maxVelocity(), 1e-9);
        assertEquals(0.0, unknown.maxAcceleration(), 1e-9);
    }

    @Test
    void timedRunnerExampleRunsOnlyWhenDue() {
        AtomicInteger runs = new AtomicInteger();
        TimedRunner<Runnable> runner = MotionLimitsExamples.every20Ms(runs::incrementAndGet);

        assertTrue(runner.shouldRunSeconds(0.00));
        runner.run(Runnable::run, 0.00);
        assertEquals(1, runs.get());

        assertFalse(runner.shouldRunSeconds(0.01));
        assertTrue(runner.shouldRunSeconds(0.02));
    }
}
