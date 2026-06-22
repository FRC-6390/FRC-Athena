package ca.frc6390.athena.hardware.input;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class InputConfigTest {
    @Test
    void lowersDigitalInput() {
        InputSpec spec = InputConfig.create()
                .digital(0)
                .toSpec("intake", "beamBreak");

        assertEquals("intake.beamBreak", spec.path());
        assertEquals(InputType.BOOLEAN, spec.type());
        assertEquals(InputSourceKind.DIGITAL_CHANNEL, spec.sourceKind());
        assertEquals(0, spec.channel());
    }

    @Test
    void lowersRuntimeNumberInput() {
        InputSpec spec = InputConfig.create()
                .runtimeNumber("dashboard/shooterTarget")
                .toSpec("shooter", "targetRpm");

        assertEquals(InputType.NUMBER, spec.type());
        assertEquals(InputSourceKind.RUNTIME_SUPPLIER, spec.sourceKind());
        assertEquals("dashboard/shooterTarget", spec.label());
    }

    @Test
    void lowersConstantStringInput() {
        InputSpec spec = InputConfig.create()
                .constant("auto")
                .toSpec("robot", "mode");

        assertEquals(InputType.STRING, spec.type());
        assertEquals(InputSourceKind.CONSTANT, spec.sourceKind());
        assertEquals("auto", spec.label());
    }

    @Test
    void requiresSourceBeforeLowering() {
        assertThrows(IllegalStateException.class, () -> InputConfig.create().toSpec("bad", "input"));
    }
}
