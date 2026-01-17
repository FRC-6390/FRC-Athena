package ca.frc6390.athena.sensors.camera.photonvision.sim;

import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.core.sim.RobotVisionSim;
import ca.frc6390.athena.core.sim.RobotVisionSimProvider;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class PhotonVisionSimProvider implements RobotVisionSimProvider {

    @Override
    public boolean supports(RobotVision vision) {
        for (VisionCamera camera : vision.getCameras().values()) {
            if (camera.supports(VisionCameraCapability.PHOTON_VISION_CAMERA)
                    || camera.supports(VisionCameraCapability.LIMELIGHT_CAMERA)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public RobotVisionSim create(RobotVision vision, AprilTagFieldLayout layout) {
        return new PhotonVisionRobotVisionSim(vision, layout);
    }
}
