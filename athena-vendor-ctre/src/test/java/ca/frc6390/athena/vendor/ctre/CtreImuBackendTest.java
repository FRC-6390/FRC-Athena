package ca.frc6390.athena.vendor.ctre;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.hardware.imu.ImuConfig;

class CtreImuBackendTest {
    private final CtreImuBackend backend = new CtreImuBackend();

    @Test
    void supportsPigeon2Only() {
        assertTrue(backend.supports(AthenaImu.PIGEON_2));
        assertFalse(backend.supports(AthenaImu.NAVX));
        assertFalse(backend.supports(AthenaImu.SIM));
    }

    @Test
    void createsDeviceWithPigeonSpec() {
        var spec = ImuConfig.create()
                .hardware(AthenaImu.PIGEON_2, 30)
                .canbus("canivore")
                .mountPose(0.0, 1.0, 2.0)
                .toSpec("robot", "pigeon");

        var device = new CtrePigeon2Device(spec, new RecordingPigeon2());

        assertEquals(spec, device.spec());
        assertEquals("canivore", device.spec().canbus());
    }

    @Test
    void deviceReadsAndControlsPigeon2Controller() {
        var spec = ImuConfig.create()
                .hardware(AthenaImu.PIGEON_2, 30)
                .toSpec("robot", "pigeon");
        var controller = new RecordingPigeon2();
        var device = new CtrePigeon2Device(spec, controller);

        controller.yaw = 91.5;
        controller.pitch = -4.0;
        controller.roll = 8.25;
        device.zeroYaw();
        device.reset();
        device.close();

        assertEquals(91.5, device.yawDegrees(), 1.0e-9);
        assertEquals(-4.0, device.pitchDegrees(), 1.0e-9);
        assertEquals(8.25, device.rollDegrees(), 1.0e-9);
        assertEquals(0.0, controller.setYaw, 1.0e-9);
        assertTrue(controller.reset);
        assertTrue(controller.closed);
    }

    private static final class RecordingPigeon2 implements CtrePigeon2Device.Pigeon2Controller {
        private double yaw;
        private double pitch;
        private double roll;
        private double setYaw = Double.NaN;
        private boolean reset;
        private boolean closed;

        @Override
        public double yawDegrees() {
            return yaw;
        }

        @Override
        public double pitchDegrees() {
            return pitch;
        }

        @Override
        public double rollDegrees() {
            return roll;
        }

        @Override
        public void setYawDegrees(double yawDegrees) {
            setYaw = yawDegrees;
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
}
