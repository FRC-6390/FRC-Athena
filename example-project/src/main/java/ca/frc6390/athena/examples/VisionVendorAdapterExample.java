package ca.frc6390.athena.examples;

import java.util.List;

import ca.frc6390.athena.vendor.limelight.LimelightCameraAdapter;
import ca.frc6390.athena.vendor.limelight.LimelightTarget;
import ca.frc6390.athena.vendor.photonvision.PhotonVisionCameraAdapter;
import ca.frc6390.athena.vendor.photonvision.PhotonVisionTarget;
import ca.frc6390.athena.vision.spec.VisionFrame;

/**
 * Example camera vendor adapter usage.
 */
public final class VisionVendorAdapterExample {
    private VisionVendorAdapterExample() {
    }

    /**
     * Converts PhotonVision-shaped target data into Athena observations.
     *
     * @return vision frame
     */
    public static VisionFrame photonFrame() {
        return PhotonVisionCameraAdapter.frameFromTargets(List.of(
                PhotonVisionTarget.aprilTag(3, 8.5, -2.0, 4.1, 0.35),
                PhotonVisionTarget.aprilTag(7, -3.2, -1.5, 2.4, 0.08)));
    }

    /**
     * Converts Limelight-shaped target data into Athena observations.
     *
     * @return vision frame
     */
    public static VisionFrame limelightFrame() {
        return LimelightCameraAdapter.frameFromTarget(
                LimelightTarget.aprilTag(7, -3.2, -1.5, 2.4, 0.92));
    }

    /**
     * Reads a Limelight-backed adapter through its real NetworkTables read path.
     *
     * @param adapter Limelight adapter
     * @return latest vision frame
     */
    public static VisionFrame latestLimelightFrame(LimelightCameraAdapter adapter) {
        return adapter.latestFrame();
    }
}
