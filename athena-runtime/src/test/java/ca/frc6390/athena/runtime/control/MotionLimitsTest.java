package ca.frc6390.athena.runtime.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

class MotionLimitsTest {
    @Test
    void driveLimitsResolveToMostConservativeFiniteValues() {
        MotionLimits limits = new MotionLimits()
                .baseDriveLimits(new MotionLimits.DriveLimits(4.5, 3.0, 2.2, 7.0))
                .driveProvider(() -> new MotionLimits.DriveLimits(3.2, 4.0, 1.9, 6.0))
                .driveProvider(() -> new MotionLimits.DriveLimits(Double.NaN, 2.5, 0.0, 5.0));

        MotionLimits.DriveLimits resolved = limits.resolveDrive();

        assertEquals(3.2, resolved.maxLinearVelocity(), 1.0e-9);
        assertEquals(2.5, resolved.maxLinearAcceleration(), 1.0e-9);
        assertEquals(1.9, resolved.maxAngularVelocity(), 1.0e-9);
        assertEquals(5.0, resolved.maxAngularAcceleration(), 1.0e-9);
    }

    @Test
    void axisLimitsResolveAndUnknownAxisReturnsNone() {
        MotionLimits limits = new MotionLimits()
                .baseAxisLimits("arm", new MotionLimits.AxisLimits(2.0, 3.0))
                .axisProvider("arm", () -> new MotionLimits.AxisLimits(1.5, 4.0))
                .axisProvider("arm", () -> new MotionLimits.AxisLimits(Double.NaN, 2.0));

        MotionLimits.AxisLimits arm = limits.resolveAxis("arm");
        MotionLimits.AxisLimits unknown = limits.resolveAxis("wrist");

        assertEquals(1.5, arm.maxVelocity(), 1.0e-9);
        assertEquals(2.0, arm.maxAcceleration(), 1.0e-9);
        assertEquals(0.0, unknown.maxVelocity(), 1.0e-9);
        assertEquals(0.0, unknown.maxAcceleration(), 1.0e-9);
    }

    @Test
    void timedRunnerRunsOnlyWhenDueAndCanReset() {
        AtomicInteger runs = new AtomicInteger();
        TimedRunner<Runnable> runner = TimedRunner.periodicMs(runs::incrementAndGet, 20.0);

        assertTrue(runner.shouldRunSeconds(0.00));
        assertTrue(runner.run(Runnable::run, 0.00));
        assertEquals(1, runs.get());
        assertFalse(runner.shouldRunSeconds(0.01));
        assertFalse(runner.run(Runnable::run, 0.01));
        assertTrue(runner.shouldRunSeconds(0.02));

        runner.reset();

        assertTrue(runner.shouldRunSeconds(0.01));
    }

    @Test
    void timedRunnerTreatsInvalidTimeAndNonPositivePeriodAsDue() {
        AtomicInteger runs = new AtomicInteger();
        TimedRunner<Runnable> runner = new TimedRunner<>(runs::incrementAndGet, 0.0);

        assertTrue(runner.run(Runnable::run, Double.NaN));
        assertTrue(runner.run(Runnable::run, 0.0));
        assertTrue(runner.run(Runnable::run, 0.0));
        assertEquals(3, runs.get());
    }
}
