package ca.frc6390.athena.hardware.backend;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class MotorClosedLoopConfigTest {
    @Test
    void preservesEveryVoltageSemanticGain() {
        MotorClosedLoopConfig config = new MotorClosedLoopConfig(
                2, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, FocPolicy.ENABLE_IF_AVAILABLE);

        assertEquals(2, config.slot());
        assertEquals(1.0, config.p(), 1.0e-9);
        assertEquals(2.0, config.i(), 1.0e-9);
        assertEquals(3.0, config.d(), 1.0e-9);
        assertEquals(4.0, config.iZone(), 1.0e-9);
        assertEquals(5.0, config.staticFeedforward(), 1.0e-9);
        assertEquals(6.0, config.velocityFeedforward(), 1.0e-9);
        assertEquals(7.0, config.accelerationFeedforward(), 1.0e-9);
        assertEquals(8.0, config.gravityFeedforward(), 1.0e-9);
    }

    @Test
    void rejectsInvalidGainsAndMalformedRoutes() {
        assertThrows(IllegalArgumentException.class, () -> new MotorClosedLoopConfig(
                0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, null));
        assertThrows(IllegalArgumentException.class, () -> new MotorClosedLoopConfig(
                0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, null));
        assertThrows(IllegalArgumentException.class, () -> new MotorClosedLoopRequest(
                ControlRoute.DEVICE_CLOSED_LOOP,
                MotorClosedLoopConfig.empty(),
                1.0));
        assertThrows(IllegalArgumentException.class, () -> MotorClosedLoopRequest.hybrid(
                MotorClosedLoopConfig.empty(),
                Double.POSITIVE_INFINITY));
    }
}
