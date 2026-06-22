package ca.frc6390.athena.hardware.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.EncoderId;

class EncoderConfigTest {
    @Test
    void lowersExplicitEncoderToSpec() {
        var spec = EncoderConfig.create()
                .hardware(AthenaEncoder.CANCODER, 22)
                .canbus("canivore")
                .absolutePosition()
                .gearRatio(2.5)
                .offset(91.4)
                .toSpec("arm", "absolute");

        assertEquals("arm.absolute", spec.path());
        assertEquals(AthenaEncoder.CANCODER, spec.kind());
        assertEquals(22, spec.id());
        assertEquals("canivore", spec.canbus());
        assertEquals(EncoderSignalType.ABSOLUTE_POSITION, spec.signalType());
        assertEquals(2.5, spec.gearRatio());
        assertEquals(91.4, spec.offset());
    }

    @Test
    void lowersAliasToSpec() {
        EncoderId id = EncoderId.of(AthenaEncoder.SIM, 2);

        var spec = EncoderConfig.create()
                .hardware(id)
                .velocity()
                .toSpec("shooter", "flywheel");

        assertEquals(AthenaEncoder.SIM, spec.kind());
        assertEquals(2, spec.id());
        assertEquals(EncoderSignalType.VELOCITY, spec.signalType());
    }

    @Test
    void requiresHardwareKindBeforeLowering() {
        assertThrows(IllegalStateException.class, () -> EncoderConfig.create().toSpec("bad", "encoder"));
    }
}
