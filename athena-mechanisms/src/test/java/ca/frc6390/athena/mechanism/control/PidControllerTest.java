package ca.frc6390.athena.mechanism.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class PidControllerTest {
    @Test
    void calculatesProportionalIntegralAndDerivativeTerms() {
        PidController controller = PidGains.of(2.0, 1.0, 0.5).controller();

        assertEquals(2.1, controller.calculate(0.0, 1.0, 0.1), 1.0e-9);
        assertEquals(4.3, controller.calculate(0.0, 2.0, 0.1), 1.0e-9);
    }

    @Test
    void continuousControllerTakesShortestAngularPath() {
        PidController controller = PidGains.of(1.0, 0.0, 0.0)
                .controller()
                .continuous(-Math.PI, Math.PI);

        assertEquals(Math.toRadians(2.0), controller.calculate(
                Math.toRadians(179.0), Math.toRadians(-179.0), 0.02), 1.0e-9);
    }

    @Test
    void validatesContinuousRange() {
        PidController controller = PidGains.of(1.0, 0.0, 0.0).controller();
        assertThrows(IllegalArgumentException.class, () -> controller.continuous(1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> controller.outputRange(1.0, 1.0));
    }
}
