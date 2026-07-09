package ca.frc6390.athena.vendor.studica;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import org.junit.jupiter.api.Test;

class StudicaImuBackendTest {
    @Test
    void supportsNavxKindsOnly() {
        StudicaImuBackend backend = new StudicaImuBackend(device -> new NoopImuHandle(device));

        assertTrue(backend.supports(ImuKinds.NAVX));
        assertTrue(backend.supports((ImuKind) () -> "studica:navx"));
        assertFalse(backend.supports((ImuKind) () -> "ctre:pigeon-2"));
    }

    @Test
    void createUsesInjectedHandleFactory() {
        ImuDevice device = ImuDevice.of(ImuKinds.NAVX, 0);
        NoopImuHandle handle = new NoopImuHandle(device);
        StudicaImuBackend backend = new StudicaImuBackend(ignored -> handle);

        assertSame(handle, backend.create(device));
    }

    @Test
    void namedNavxPortsCreateClearDeclarations() {
        assertEquals(0, StudicaImus.navx(StudicaNavxPort.MXP_SPI).id());
        assertEquals(1, StudicaImus.navx(StudicaNavxPort.MXP_UART).id());
        assertEquals(2, StudicaImus.navx(StudicaNavxPort.USB1).id());
        assertEquals(3, StudicaImus.navx(StudicaNavxPort.USB2).id());
        assertEquals(4, StudicaImus.navx(StudicaNavxPort.I2C).id());
        assertEquals(ImuKinds.NAVX, StudicaImus.navx(null).kind());
    }

    @Test
    void handleDelegatesYawAngleResetZeroAndClose() {
        RecordingNavxController controller = new RecordingNavxController();
        ImuHandle genericHandle = new StudicaImuHandle(StudicaImus.navx(StudicaNavxPort.USB1), controller);
        StudicaImuHandle handle = (StudicaImuHandle) genericHandle;

        genericHandle.activate();
        assertEquals(12.0, genericHandle.yawDegrees(), 1.0e-9);
        assertEquals(34.0, genericHandle.angleDegrees(), 1.0e-9);
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
