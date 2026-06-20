package ca.frc6390.athena.vendor.ctre;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderSignalType;

class CtreEncoderBackendTest {
    private final CtreEncoderBackend backend = new CtreEncoderBackend();

    @Test
    void supportsCancoderOnly() {
        assertTrue(backend.supports(AthenaEncoder.CANCODER));
        assertFalse(backend.supports(AthenaEncoder.SIM));
    }

    @Test
    void createsDeviceWithCancoderSpec() {
        var spec = EncoderConfig.create()
                .hardware(AthenaEncoder.CANCODER, 22)
                .canbus("canivore")
                .absolutePosition()
                .toSpec("swerve.frontLeft", "steer");

        var device = new CtreEncoderDevice(spec, new RecordingCANCoder(0.0, 0.0, 0.0));

        assertEquals(spec, device.spec());
        assertEquals(EncoderSignalType.ABSOLUTE_POSITION, device.spec().signalType());
    }

    @Test
    void deviceReadsPositionAbsolutePositionAndVelocity() {
        var spec = EncoderConfig.create()
                .hardware(AthenaEncoder.CANCODER, 22)
                .toSpec("swerve.frontLeft", "steer");
        var controller = new RecordingCANCoder(4.25, 0.75, 12.5);
        var device = new CtreEncoderDevice(spec, controller);

        assertEquals(4.25, device.positionRotations(), 1.0e-9);
        assertEquals(0.75, device.absolutePositionRotations(), 1.0e-9);
        assertEquals(12.5, device.velocityRotationsPerSecond(), 1.0e-9);
    }

    private record RecordingCANCoder(
            double positionRotations,
            double absolutePositionRotations,
            double velocityRotationsPerSecond)
            implements CtreEncoderDevice.CANCoderController {
    }
}
