package ca.frc6390.athena.mechanism.motion;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MotionProfileRuntimeTest {
    @Test
    void reachesForwardAndReverseGoalsWithoutExceedingMotionLimits() {
        assertProfile(0.0, 4.0);
        assertProfile(4.0, -1.0);
    }

    @Test
    void startsFromMeasuredPositionWithoutImportingUnsafeMeasuredVelocity() {
        MotionProfileRuntime runtime = new MotionProfileRuntime(new MotionProfile(2.0, 2.0));
        MotionReference previous = new MotionReference(0.9, 1.0, 0.0);

        for (int cycle = 0; cycle < 100; cycle++) {
            MotionReference next = runtime.step(
                    previous.position(), previous.velocity(), 1.0, 0.02);
            assertTrue(Math.abs(next.velocity()) <= 2.0 + 1.0e-9);
            assertTrue(Math.abs(next.acceleration()) <= 2.0 + 1.0e-9);
            previous = next;
        }

        assertEquals(1.0, previous.position(), 1.0e-9);
        assertEquals(0.0, previous.velocity(), 1.0e-9);
    }

    private static void assertProfile(double start, double goal) {
        MotionProfileRuntime runtime = new MotionProfileRuntime(new MotionProfile(1.5, 3.0));
        MotionReference previous = new MotionReference(start, 0.0, 0.0);
        double direction = Math.signum(goal - start);

        for (int cycle = 0; cycle < 300; cycle++) {
            MotionReference next = runtime.step(
                    previous.position(), previous.velocity(), goal, 0.02);
            assertTrue(Math.abs(next.velocity()) <= 1.5 + 1.0e-9);
            assertTrue(Math.abs(next.acceleration()) <= 3.0 + 1.0e-9);
            assertTrue((next.position() - previous.position()) * direction >= -1.0e-9);
            previous = next;
        }

        assertEquals(goal, previous.position(), 1.0e-9);
        assertEquals(0.0, previous.velocity(), 1.0e-9);
    }
}
