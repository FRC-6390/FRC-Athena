package ca.frc6390.athena.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.PoseMeasurement;
import ca.frc6390.athena.runtime.measurement.TargetMeasurement;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.ref.CameraRef;
import ca.frc6390.athena.vision.ref.LimelightPoseRef;
import ca.frc6390.athena.vision.ref.PhotonPoseRef;
import ca.frc6390.athena.vision.ref.HeliOS;
import ca.frc6390.athena.vision.ref.Limelight;
import ca.frc6390.athena.vision.ref.Photon;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;
import org.junit.jupiter.api.Test;

class CameraRefTest {
    @Test
    void vendorSpecificRefsAreCameraRefs() {
        CameraRef limelight = Limelight.camera("limelight-left")
                .mount(0.25, 0.2, 0.4, 0.0, -20.0, 0.0);
        CameraRef helios = HeliOS.camera("10.63.90.11");
        CameraRef photon = Photon.camera("front");

        assertEquals(AthenaCamera.LIMELIGHT, limelight.kind());
        assertEquals(AthenaCamera.HELIOS, helios.kind());
        assertEquals(AthenaCamera.PHOTONVISION, photon.kind());
    }

    @Test
    void vendorPoseOptionsLiveOnPoseMeasurements() {
        LimelightPoseRef limelightPose = Limelight.camera("limelight-left")
                .pose()
                .megatag2Blue()
                .tags(1, 2, 3);
        PhotonPoseRef photonPose = Photon.camera("front")
                .pose()
                .multiTagOnCoprocessor();

        assertEquals("megatag2-blue", limelightPose.poseMode());
        assertEquals(List.of(1, 2, 3), limelightPose.tags());
        assertEquals("multi-tag-coprocessor", photonPose.strategy());
    }

    @Test
    void cameraFramesProduceTargetMeasurements() {
        CameraRef camera = Cameras.sim("sim")
                .bindFrame(() -> VisionFrame.of(VisionObservation.tag(7, 4.0, 1.0, 3.0, 0.9)));

        var measurements = camera.targets().measurements();

        assertEquals(1, measurements.size());
        TargetMeasurement target = assertInstanceOf(TargetMeasurement.class, measurements.get(0));
        assertEquals(7, target.targetId());
        assertEquals(3.0, target.distanceMeters(), 1.0e-9);
    }

    @Test
    void camerasExposePoseMeasurementsForLocalization() {
        CameraRef camera = Cameras.helios("helios")
                .bindPose(() -> List.of(new PoseMeasurement(
                        new PoseSnapshot(1.0, 2.0, 0.0),
                        null,
                        5.0,
                        0.1,
                        0.0,
                        2,
                        null,
                        null)));

        assertTrue(camera.pose().latestMeasurement().orElseThrow() instanceof PoseMeasurement);
    }
}
