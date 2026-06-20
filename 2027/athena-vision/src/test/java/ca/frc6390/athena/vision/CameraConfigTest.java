package ca.frc6390.athena.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.api.hardware.CameraId;
import ca.frc6390.athena.vision.config.CameraConfig;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.spec.CameraMountPose;

class CameraConfigTest {
    @Test
    void lowersCameraConfig() {
        var spec = CameraConfig.create()
                .hardware(AthenaCamera.PHOTONVISION, "frontCam")
                .mountPose(0.25, 0.1, 0.55, 0.0, -18.0, 0.0)
                .toSpec("robot", "front");

        assertEquals("robot.front", spec.path());
        assertEquals(AthenaCamera.PHOTONVISION, spec.kind());
        assertEquals("frontCam", spec.cameraName());
        assertEquals(new CameraMountPose(0.25, 0.1, 0.55, 0.0, -18.0, 0.0), spec.mountPose());
        assertFalse(spec.validate().hasErrors());
    }

    @Test
    void aliasCarriesCameraName() {
        var spec = Cameras.camera("driver", camera -> camera.hardware(CameraId.of(AthenaCamera.LIMELIGHT, "limelight-front")));

        assertEquals("vision.driver", spec.path());
        assertEquals(AthenaCamera.LIMELIGHT, spec.kind());
        assertEquals("limelight-front", spec.cameraName());
    }

    @Test
    void invalidMountPoseReportsError() {
        var spec = CameraConfig.create()
                .hardware(AthenaCamera.SIM, "simCam")
                .mountPose(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0)
                .toSpec("vision", "sim");

        assertTrue(spec.validate().hasErrors());
        assertFalse(spec.validate().errorsWithCode("vision.invalid-mount-pose").isEmpty());
    }

    @Test
    void hardwareKindIsRequired() {
        assertThrows(IllegalStateException.class, () -> CameraConfig.create().toSpec("vision", "front"));
        assertThrows(NullPointerException.class, () -> CameraConfig.create().hardware((CameraId) null));
    }
}
