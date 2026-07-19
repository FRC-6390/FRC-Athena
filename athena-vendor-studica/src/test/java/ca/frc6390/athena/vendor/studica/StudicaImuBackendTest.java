package ca.frc6390.athena.vendor.studica;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.HardwareAddress;
import org.junit.jupiter.api.Test;

class StudicaImuBackendTest {
    @Test
    void supportsNavxKindsOnly() {
        StudicaImuBackend backend = new StudicaImuBackend(device -> new NoopImuHandle(device));

        assertTrue(backend.supports(ImuKinds.NAVX));
        assertTrue(backend.supports((ImuKind) () -> "studica:navx"));
        assertFalse(backend.supports((ImuKind) () -> "ctre:pigeon-2"));
        assertTrue(backend.supports(StudicaImus.navx(StudicaNavxPort.MXP_SPI)));
        assertTrue(backend.supports(StudicaImus.navx(StudicaNavxPort.MXP_UART)));
        assertTrue(backend.supports(StudicaImus.navx(StudicaNavxPort.USB1)));
        assertTrue(backend.supports(StudicaImus.navx(StudicaNavxPort.I2C)));
        assertFalse(backend.supports(ImuDevice.of(ImuKinds.NAVX, 0)));
    }

    @Test
    void createUsesInjectedHandleFactory() {
        ImuDevice device = StudicaImus.navx(StudicaNavxPort.MXP_SPI);
        NoopImuHandle handle = new NoopImuHandle(device);
        StudicaImuBackend backend = new StudicaImuBackend(ignored -> handle);

        assertSame(handle, backend.create(device));
    }

    @Test
    void namedNavxPortsCreateClearDeclarations() {
        assertTrue(StudicaImus.navx(StudicaNavxPort.MXP_SPI).connection() instanceof HardwareAddress.Spi);
        assertTrue(StudicaImus.navx(StudicaNavxPort.MXP_UART).connection() instanceof HardwareAddress.Serial);
        assertEquals(1, ((HardwareAddress.Usb) StudicaImus.navx(StudicaNavxPort.USB1).connection()).port());
        assertEquals(2, ((HardwareAddress.Usb) StudicaImus.navx(StudicaNavxPort.USB2).connection()).port());
        assertTrue(StudicaImus.navx(StudicaNavxPort.I2C).connection() instanceof HardwareAddress.I2c);
        assertEquals(ImuKinds.NAVX, StudicaImus.navx(null).kind());
    }

    @Test
    void handleDelegatesYawAngleResetZeroAndClose() {
        RecordingNavxController controller = new RecordingNavxController();
        ImuHandle genericHandle = new StudicaImuHandle(StudicaImus.navx(StudicaNavxPort.USB1), controller);
        StudicaImuHandle handle = (StudicaImuHandle) genericHandle;

        genericHandle.activate();
        genericHandle.refreshInputs();
        assertEquals(12.0, genericHandle.yawDegrees(), 1.0e-9);
        assertEquals(34.0, genericHandle.angleDegrees(), 1.0e-9);
        assertEquals(3.0, genericHandle.pitchDegrees(), 1.0e-9);
        assertEquals(4.0, genericHandle.rollDegrees(), 1.0e-9);
        assertEquals(5.0, genericHandle.yawRateDegreesPerSecond(), 1.0e-9);
        assertEquals(6.0, genericHandle.pitchRateDegreesPerSecond(), 1.0e-9);
        assertEquals(7.0, genericHandle.rollRateDegreesPerSecond(), 1.0e-9);
        assertEquals(0.1, genericHandle.linearAccelerationXG(), 1.0e-9);
        assertEquals(0.2, genericHandle.linearAccelerationYG(), 1.0e-9);
        assertEquals(0.3, genericHandle.linearAccelerationZG(), 1.0e-9);
        assertTrue(genericHandle.isConnected());
        assertFalse(genericHandle.isCalibrating());
        assertTrue(Double.isFinite(genericHandle.lastUpdateSeconds()));
        genericHandle.zeroYaw();
        genericHandle.reset();
        handle.close();

        assertEquals(1, controller.zeroCalls);
        assertEquals(1, controller.resetCalls);
        assertEquals(1, controller.closeCalls);
        assertEquals(1, controller.activateCalls);
    }

    private record NoopImuHandle(ImuDevice device) implements ImuHandle {
        @Override
        public double yawDegrees() {
            return 0.0;
        }
    }

    private static final class RecordingNavxController implements StudicaImuHandle.NavxController {
        private int zeroCalls;
        private int resetCalls;
        private int closeCalls;
        private int activateCalls;

        @Override
        public void activate() {
            activateCalls++;
        }

        @Override
        public double yawDegrees() {
            return 12.0;
        }

        @Override
        public double angleDegrees() {
            return 34.0;
        }

        @Override
        public double pitchDegrees() {
            return 3.0;
        }

        @Override
        public double rollDegrees() {
            return 4.0;
        }

        @Override
        public double yawRateDegreesPerSecond() {
            return 5.0;
        }

        @Override public double pitchRateDegreesPerSecond() { return 6.0; }
        @Override public double rollRateDegreesPerSecond() { return 7.0; }

        @Override
        public double linearAccelerationXG() {
            return 0.1;
        }

        @Override
        public double linearAccelerationYG() {
            return 0.2;
        }

        @Override
        public double linearAccelerationZG() {
            return 0.3;
        }

        @Override
        public void zeroYaw() {
            zeroCalls++;
        }

        @Override
        public void reset() {
            resetCalls++;
        }

        @Override
        public void close() {
            closeCalls++;
        }
    }
}
