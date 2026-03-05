package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.context.RobotScopedContext;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class RobotScopedContextTimingTest {

    private static final RobotScopedContext CONTEXT = new RobotScopedContext() {
        @Override
        public RobotCore<?> robotCore() {
            return null;
        }
    };

    @AfterEach
    void resetRobotTime() {
        RobotTime.resetNowSecondsForTest();
    }

    @Test
    void contextExposesRobotLoopSnapshotAndLatencyHelpers() {
        RobotTime.updateNowSeconds(100.0);
        RobotTime.updateNowSeconds(100.02);

        assertEquals(2L, CONTEXT.robotLoopCycleCount());
        assertEquals(0.02, CONTEXT.robotLoopDtSeconds(), 1e-9);
        assertEquals(100.02, CONTEXT.robotLoopSnapshot().nowSeconds(), 1e-9);

        double now = CONTEXT.robotNowSeconds();
        assertTrue(now >= 100.02);
        assertTrue(now <= 100.27);

        double effectiveLatency = CONTEXT.effectiveInputLatencySeconds(
                now + 10.0,
                0.02,
                0.005,
                1.0);
        assertEquals(0.015, effectiveLatency, 1e-9);

        double predictiveLead = CONTEXT.predictiveLeadSeconds(effectiveLatency, 0.01, 0.08);
        assertEquals(0.025, predictiveLead, 1e-9);
    }
}
