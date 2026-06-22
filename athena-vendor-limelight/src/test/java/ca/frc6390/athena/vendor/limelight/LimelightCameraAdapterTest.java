package ca.frc6390.athena.vendor.limelight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.vision.config.Cameras;

class LimelightCameraAdapterTest {
    private final RecordingLimelightClient client = new RecordingLimelightClient(LimelightTarget.noTarget());
    private final LimelightCameraAdapter adapter = new LimelightCameraAdapter(client);

    @Test
    void supportsLimelightCamerasOnly() {
        var limelight = Cameras.camera("front", camera -> camera.hardware(AthenaCamera.LIMELIGHT, "limelight"));
        var photon = Cameras.camera("front", camera -> camera.hardware(AthenaCamera.PHOTONVISION, "frontCam"));

        assertTrue(adapter.supports(limelight));
        assertFalse(adapter.supports(photon));
    }

    @Test
    void convertsTargetIntoVisionFrame() {
        var frame = adapter.frame(LimelightTarget.aprilTag(7, -3.2, -1.5, 2.4, 0.92));

        assertEquals(7, frame.tagId().orElseThrow());
        assertEquals(-3.2, frame.yawDegrees().orElseThrow(), 1.0e-9);
        assertEquals(0.92, frame.bestTarget().orElseThrow().confidence(), 1.0e-9);
    }

    @Test
    void noTargetCreatesNoTargetFrame() {
        assertFalse(LimelightCameraAdapter.frameFromTarget(LimelightTarget.noTarget()).hasValidTarget());
    }

    @Test
    void readsLatestTargetFromClient() {
        client.target = LimelightTarget.aprilTag(4, 6.5, -2.0, 3.25, 1.8);

        var frame = adapter.latestFrame();

        assertEquals(4, frame.tagId().orElseThrow());
        assertEquals(6.5, frame.yawDegrees().orElseThrow(), 1.0e-9);
        assertEquals(-2.0, frame.pitchDegrees().orElseThrow(), 1.0e-9);
        assertEquals(3.25, frame.distanceMeters().orElseThrow(), 1.0e-9);
        assertEquals(1.8, frame.bestTarget().orElseThrow().confidence(), 1.0e-9);
    }

    @Test
    void noTargetFromClientCreatesNoTargetFrame() {
        client.target = LimelightTarget.noTarget();

        assertFalse(adapter.latestFrame().hasValidTarget());
    }

    private static final class RecordingLimelightClient implements LimelightCameraAdapter.LimelightClient {
        private LimelightTarget target;

        private RecordingLimelightClient(LimelightTarget target) {
            this.target = target;
        }

        @Override
        public LimelightTarget latestTarget() {
            return target;
        }
    }
}
