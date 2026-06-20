package ca.frc6390.athena.vendor.photonvision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.vision.config.Cameras;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

class PhotonVisionCameraAdapterTest {
    private final PhotonVisionCameraAdapter adapter = new PhotonVisionCameraAdapter();

    @Test
    void supportsPhotonVisionCamerasOnly() {
        var photon = Cameras.camera("front", camera -> camera.hardware(AthenaCamera.PHOTONVISION, "frontCam"));
        var limelight = Cameras.camera("front", camera -> camera.hardware(AthenaCamera.LIMELIGHT, "limelight"));

        assertTrue(adapter.supports(photon));
        assertFalse(adapter.supports(limelight));
    }

    @Test
    void convertsTargetsIntoVisionFrame() {
        var frame = adapter.frame(List.of(
                PhotonVisionTarget.aprilTag(3, 8.5, -2.0, 4.1, 0.4),
                PhotonVisionTarget.aprilTag(7, -3.2, -1.5, 2.4, 0.08)));

        assertEquals(7, frame.tagId().orElseThrow());
        assertEquals(0.92, frame.bestTarget().orElseThrow().confidence(), 1.0e-9);
    }

    @Test
    void emptyTargetListCreatesNoTargetFrame() {
        assertFalse(PhotonVisionCameraAdapter.frameFromTargets(List.of()).hasValidTarget());
    }

    @Test
    void convertsRealPhotonPipelineResultIntoVisionFrame() {
        var target = new PhotonTrackedTarget(
                5.0,
                -1.5,
                12.0,
                0.0,
                4,
                -1,
                0.0f,
                new Transform3d(3.0, 4.0, 0.0, edu.wpi.first.math.geometry.Rotation3d.kZero),
                new Transform3d(),
                0.25,
                corners(),
                corners());
        var result = new PhotonPipelineResult(0L, 0L, 0L, 0L, List.of(target), Optional.empty());

        var frame = PhotonVisionCameraAdapter.frameFromResult(result);

        assertTrue(frame.hasValidTarget());
        assertEquals(4, frame.tagId().orElseThrow());
        assertEquals(5.0, frame.yawDegrees().orElseThrow(), 1.0e-9);
        assertEquals(-1.5, frame.pitchDegrees().orElseThrow(), 1.0e-9);
        assertEquals(5.0, frame.distanceMeters().orElseThrow(), 1.0e-9);
        assertEquals(0.75, frame.bestTarget().orElseThrow().confidence(), 1.0e-9);
    }

    @Test
    void readsLatestFrameThroughPhotonClient() {
        var spec = Cameras.camera("front", camera -> camera.hardware(AthenaCamera.PHOTONVISION, "frontCam"));
        var target = new PhotonTrackedTarget(
                -2.0,
                1.0,
                9.0,
                0.0,
                8,
                -1,
                0.0f,
                new Transform3d(1.0, 2.0, 2.0, edu.wpi.first.math.geometry.Rotation3d.kZero),
                new Transform3d(),
                0.1,
                corners(),
                corners());
        var client = new RecordingPhotonClient(new PhotonPipelineResult(0L, 0L, 0L, 0L, List.of(target), Optional.empty()));
        var adapter = new PhotonVisionCameraAdapter(spec, client);

        var frame = adapter.latestFrame();
        adapter.close();

        assertEquals(spec, adapter.spec());
        assertEquals(8, frame.tagId().orElseThrow());
        assertEquals(3.0, frame.distanceMeters().orElseThrow(), 1.0e-9);
        assertTrue(client.closed);
    }

    private static final class RecordingPhotonClient implements PhotonVisionCameraAdapter.PhotonClient {
        private final PhotonPipelineResult result;
        private boolean closed;

        private RecordingPhotonClient(PhotonPipelineResult result) {
            this.result = result;
        }

        @Override
        public List<PhotonPipelineResult> unreadResults() {
            return List.of(result);
        }

        @Override
        public void close() {
            closed = true;
        }
    }

    private static List<TargetCorner> corners() {
        return List.of(
                new TargetCorner(0.0, 0.0),
                new TargetCorner(1.0, 0.0),
                new TargetCorner(1.0, 1.0),
                new TargetCorner(0.0, 1.0));
    }
}
