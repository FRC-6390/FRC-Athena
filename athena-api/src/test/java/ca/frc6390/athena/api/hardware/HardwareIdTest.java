package ca.frc6390.athena.api.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class HardwareIdTest {
    @Test
    void motorIdDefaultsToRioBus() {
        MotorId id = MotorId.of(AthenaMotor.SIM, 1);

        assertEquals(AthenaMotor.SIM, id.kind());
        assertEquals(1, id.id());
        assertEquals("rio", id.canbus());
    }

    @Test
    void motorIdCanMoveToNamedBus() {
        MotorId id = MotorId.of(AthenaMotor.TALON_FX, 12).canbus("canivore");

        assertEquals("canivore", id.canbus());
    }

    @Test
    void encoderIdDefaultsToRioBus() {
        EncoderId id = EncoderId.of(AthenaEncoder.SIM, 2);

        assertEquals(AthenaEncoder.SIM, id.kind());
        assertEquals(2, id.id());
        assertEquals("rio", id.canbus());
    }

    @Test
    void encoderIdCanMoveToNamedBus() {
        EncoderId id = EncoderId.of(AthenaEncoder.CANCODER, 22).canbus("canivore");

        assertEquals("canivore", id.canbus());
    }

    @Test
    void imuIdDefaultsToRioBus() {
        ImuId id = ImuId.of(AthenaImu.SIM, 0);

        assertEquals(AthenaImu.SIM, id.kind());
        assertEquals(0, id.id());
        assertEquals("rio", id.canbus());
    }

    @Test
    void imuIdCanMoveToNamedBus() {
        ImuId id = ImuId.of(AthenaImu.PIGEON_2, 30).canbus("canivore");

        assertEquals("canivore", id.canbus());
    }

    @Test
    void cameraIdDefaultsBlankName() {
        CameraId id = CameraId.of(AthenaCamera.SIM, " ");

        assertEquals(AthenaCamera.SIM, id.kind());
        assertEquals("camera", id.name());
    }

    @Test
    void cameraIdPreservesNamedCamera() {
        CameraId id = CameraId.of(AthenaCamera.PHOTONVISION, "frontCam");

        assertEquals("frontCam", id.name());
    }

    @Test
    void idsRequireKinds() {
        assertThrows(NullPointerException.class, () -> MotorId.of(null, 1));
        assertThrows(NullPointerException.class, () -> EncoderId.of(null, 1));
        assertThrows(NullPointerException.class, () -> ImuId.of(null, 1));
        assertThrows(NullPointerException.class, () -> CameraId.of(null, "camera"));
    }
}
