package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

final class RobotAutoInputOverridesTest {

    @Test
    void runtimeInputOverrideTakesPrecedenceUntilCleared() {
        RobotAuto autos = new RobotAuto();
        autos.inputs().string("handoff.mode", () -> "default");

        assertEquals("default", autos.inputs().string("handoff.mode"));

        autos.inputs().string("handoff.mode", "amp");
        assertEquals("amp", autos.inputs().string("handoff.mode"));

        autos.inputs().resetString("handoff.mode");
        assertEquals("default", autos.inputs().string("handoff.mode"));
    }

    @Test
    void clearAllRuntimeInputOverridesRestoresRegisteredSuppliers() {
        RobotAuto autos = new RobotAuto();
        autos.inputs().integer("handoff.count", () -> 0);
        autos.inputs().bool("handoff.fire", () -> false);

        autos.inputs().integer("handoff.count", 3);
        autos.inputs().bool("handoff.fire", true);
        assertEquals(3, autos.inputs().integer("handoff.count"));
        assertTrue(autos.inputs().bool("handoff.fire"));

        autos.inputs().clear();
        assertEquals(0, autos.inputs().integer("handoff.count"));
        assertEquals(false, autos.inputs().bool("handoff.fire"));
    }

    @Test
    void runtimeInputOverrideTypeMismatchFailsWhenRead() {
        RobotAuto autos = new RobotAuto();
        autos.inputs().integer("handoff.shared", () -> 1);
        autos.inputs().bool("handoff.shared", true);

        IllegalStateException ex = assertThrows(
                IllegalStateException.class,
                () -> autos.inputs().integer("handoff.shared"));
        assertTrue(ex.getMessage().contains("Runtime auto input type mismatch"));
    }
}
