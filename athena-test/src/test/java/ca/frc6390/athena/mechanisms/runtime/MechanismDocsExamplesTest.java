package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.Mechanisms;

final class MechanismDocsExamplesTest {

    @Test
    void disableAllHooksAndControlLoopsMatchesDocsExample() {
        Mechanism mechanism = MechanismDefinitions.build(Mechanisms.create("docs").definition());

        assertTrue(mechanism.hooksEnabled());
        assertTrue(mechanism.controlLoopsEnabled());

        mechanism.control().disableAllHooksAndControlLoops();

        assertFalse(mechanism.hooksEnabled());
        assertFalse(mechanism.controlLoopsEnabled());
    }

    @Test
    void testModeSuppressesPidOutput() {
        Mechanism mechanism = MechanismDefinitions.build(
                Mechanisms.create("docs-loop")
                        .behavior(behavior -> behavior.control(control -> control
                                .customLoop("pidLike", loop -> loop.custom(ctx -> 1.0))))
                        .definition());

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
