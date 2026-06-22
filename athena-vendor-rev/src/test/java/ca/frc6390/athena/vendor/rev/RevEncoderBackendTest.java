package ca.frc6390.athena.vendor.rev;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderSignalType;

class RevEncoderBackendTest {
    private final RevEncoderBackend backend = new RevEncoderBackend();

    @Test
    void supportsRevThroughBoreOnly() {
        assertTrue(backend.supports(AthenaEncoder.REV_THROUGH_BORE));
        assertFalse(backend.supports(AthenaEncoder.CANCODER));
    }

    @Test
    void createsDeviceWithThroughBoreSpec() {
        var spec = EncoderConfig.create()
                .hardware(AthenaEncoder.REV_THROUGH_BORE, 7)
                .absolutePosition()
                .toSpec("arm", "absolute");

        var device = new RevThroughBoreEncoderDevice(spec, new RecordingThroughBore(0.0));

        assertEquals(spec, device.spec());
        assertEquals(EncoderSignalType.ABSOLUTE_POSITION, device.spec().signalType());
    }

    @Test
    void deviceReadsAbsolutePositionFromDutyCycleController() {
        var spec = EncoderConfig.create()
                .hardware(AthenaEncoder.REV_THROUGH_BORE, 7)
                .absolutePosition()
                .toSpec("arm", "absolute");
        var device = new RevThroughBoreEncoderDevice(spec, new RecordingThroughBore(0.625));

        assertEquals(0.625, device.absolutePositionRotations(), 1.0e-9);
        assertEquals(0.625, device.positionRotations(), 1.0e-9);
        assertThrows(UnsupportedOperationException.class, device::velocityRotationsPerSecond);
    }

    private record RecordingThroughBore(double absolutePositionRotations)
            implements RevThroughBoreEncoderDevice.ThroughBoreController {
        @Override
        public double velocityRotationsPerSecond() {
            throw new UnsupportedOperationException("No velocity in test controller.");
        }
    }
}
