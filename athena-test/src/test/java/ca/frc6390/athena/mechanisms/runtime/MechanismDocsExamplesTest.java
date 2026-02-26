package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

final class MechanismDocsExamplesTest {

    @Test
    void disableAllHooksAndControlLoopsMatchesDocsExample() {
        Mechanism mechanism = MechanismConfig.<Mechanism>generic().build();

        assertTrue(mechanism.hooksEnabled());
        assertTrue(mechanism.controlLoopsEnabled());

        mechanism.control().disableAllHooksAndControlLoops();

        assertFalse(mechanism.hooksEnabled());
        assertFalse(mechanism.controlLoopsEnabled());
    }

    @Test
    void testModeSuppressesPidOutput() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c.controlLoop("pidLike", 20.0, ctx -> 1.0));
        Mechanism mechanism = cfg.build();

        setRobotMode(mechanism, "TELE");
        mechanism.update();
        assertNotEquals(0.0, mechanism.output(), 1e-9);

        setRobotMode(mechanism, "TEST");
        mechanism.update();
        assertEquals(0.0, mechanism.output(), 1e-9);
    }

    private static void setRobotMode(Mechanism mechanism, String mode) {
        mechanism.setRobotModeForTest(mode);
    }
}
