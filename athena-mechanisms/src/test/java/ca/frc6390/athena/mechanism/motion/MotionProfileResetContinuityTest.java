package ca.frc6390.athena.mechanism.motion;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class MotionProfileResetContinuityTest {
    @Test
    void resetPreservesFiniteVelocityWithinProfileLimits() {
        MotionProfileRuntime runtime = new MotionProfileRuntime(new MotionProfile(2.0, 4.0));

        runtime.reset(1.5, -1.25);

        assertEquals(1.5, runtime.reference().position(), 1.0e-9);
        assertEquals(-1.25, runtime.reference().velocity(), 1.0e-9);
        assertEquals(0.0, runtime.reference().acceleration(), 1.0e-9);
    }

    @Test
    void resetClampsUnsafeVelocityAndRejectsNonFiniteInputs() {
        MotionProfileRuntime runtime = new MotionProfileRuntime(new MotionProfile(2.0, 4.0));

        runtime.reset(0.0, 5.0);
        assertEquals(2.0, runtime.reference().velocity(), 1.0e-9);
        assertThrows(IllegalArgumentException.class, () -> runtime.reset(Double.NaN, 0.0));
        assertThrows(IllegalArgumentException.class, () -> runtime.reset(0.0, Double.POSITIVE_INFINITY));
    }
}
