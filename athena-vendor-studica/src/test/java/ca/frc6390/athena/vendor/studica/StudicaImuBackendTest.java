package ca.frc6390.athena.vendor.studica;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.hardware.backend.ImuDevice;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuSpec;

class StudicaImuBackendTest {
    private final StudicaImuBackend backend = new StudicaImuBackend();

    @Test
    void supportsNavxOnly() {
        assertTrue(backend.supports(AthenaImu.NAVX));
        assertFalse(backend.supports(AthenaImu.PIGEON_2));
    }

    @Test
    void createsDeviceFromNormalizedImuSpec() {
        var spec = ImuConfig.create()
                .hardware(AthenaImu.NAVX, 0)
                .mountPose(2.0, 0.0, 1.0)
                .toSpec("robot", "navx");

        var backend = new StudicaImuBackend(FakeImuDevice::new);
        var device = assertInstanceOf(FakeImuDevice.class, backend.create(spec));

        assertEquals(spec, device.spec());
    }

    @Test
    void deviceReadsAndControlsRealNavxController() {
        var spec = ImuConfig.create()
                .hardware(AthenaImu.NAVX, 0)
                .toSpec("robot", "navx");
        var controller = new RecordingNavx();
        var device = new StudicaImuDevice(spec, controller);

        controller.yaw = -45.0;
        controller.angle = 725.0;
        device.zeroYaw();
        device.reset();
        device.close();

        assertEquals(-45.0, device.yawDegrees(), 1.0e-9);
        assertEquals(725.0, device.angleDegrees(), 1.0e-9);
        assertTrue(controller.zeroed);
        assertTrue(controller.reset);
        assertTrue(controller.closed);
    }

    private static final class RecordingNavx implements StudicaImuDevice.NavxController {
        private double yaw;
        private double angle;
        private boolean zeroed;
        private boolean reset;
        private boolean closed;

        @Override
        public double yawDegrees() {
            return yaw;
        }

        @Override
        public double angleDegrees() {
            return angle;
        }

        @Override
        public void zeroYaw() {
            zeroed = true;
        }

        @Override
        public void reset() {
            reset = true;
        }

        @Override
        public void close() {
            closed = true;
        }
    }

    private record FakeImuDevice(ImuSpec spec) implements ImuDevice {
        @Override
        public double yawDegrees() {
            return 0.0;
        }
    }
}
