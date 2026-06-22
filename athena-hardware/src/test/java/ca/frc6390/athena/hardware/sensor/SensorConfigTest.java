package ca.frc6390.athena.hardware.sensor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.hardware.input.InputSourceKind;

class SensorConfigTest {
    @Test
    void limitSwitchPreservesHardstopMetadata() {
        SensorSpec spec = SensorConfig.create()
                .limitSwitch(2)
                .hardstop(BlockDirection.NEGATIVE, -12.5)
                .toSpec("arm", "lowerLimit");

        assertEquals(SensorKind.LIMIT_SWITCH, spec.kind());
        assertEquals(InputSourceKind.DIGITAL_CHANNEL, spec.input().sourceKind());
        assertEquals(2, spec.input().channel());
        assertTrue(spec.hardstop());
        assertEquals(BlockDirection.NEGATIVE, spec.blockDirection());
        assertEquals(-1, spec.blockDirection().multiplier());
        assertEquals(-12.5, spec.position());
    }

    @Test
    void buttonAndBeamBreakDefaultToInvertedTriggerSemantics() {
        SensorSpec button = SensorConfig.create().button(0).toSpec("driver", "confirm");
        SensorSpec beamBreak = SensorConfig.create().beamBreak(1).toSpec("intake", "loaded");

        assertTrue(button.inverted());
        assertTrue(beamBreak.inverted());
        assertTrue(button.triggered(false));
        assertFalse(beamBreak.triggered(true));
    }

    @Test
    void explicitInversionOverridesDefaults() {
        SensorSpec spec = SensorConfig.create()
                .beamBreak(3)
                .inverted(false)
                .toSpec("intake", "loaded");

        assertFalse(spec.inverted());
        assertTrue(spec.triggered(true));
    }

    @Test
    void requiresInputBeforeLowering() {
        assertThrows(IllegalStateException.class, () -> SensorConfig.create().toSpec("bad", "sensor"));
    }
}
