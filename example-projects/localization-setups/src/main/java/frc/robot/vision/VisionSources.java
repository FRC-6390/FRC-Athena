package frc.robot.vision;

import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.device.HeliosDevice;
import ca.frc6390.athena.vision.device.LimelightDevice;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;
import ca.frc6390.athena.vision.signal.TargetSignal;
import edu.wpi.first.math.util.Units;
import java.util.List;

public final class VisionSources {
    public final LimelightDevice frontLimelight = Cameras.limelight("limelight-front")
            .mount(new CameraMountPose(0.32, 0.18, 0.62, 0.0, -18.0, 0.0));
    public final PhotonVisionDevice rearPhoton = Cameras.photonVision("photon-rear")
            .mount(new CameraMountPose(-0.28, -0.18, 0.55, 180.0, -12.0, 0.0));
    public final HeliosDevice driverHelios = Cameras.helios("10.63.90.11")
            .mount(new CameraMountPose(0.0, 0.0, 0.7, 0.0, -20.0, 0.0));
    public final CameraDevice customCamera = Cameras.camera(
            () -> "team:target-camera", "custom-target-camera")
            .bindTargets(() -> List.of(Measurements.target(42, 3.5, -8.0, 2.7, 0.95)));

    public final PoseSignal limelightPose = frontLimelight.pose()
            .megatag2Blue()
            .tags(7, 8)
            .singleTagStdDevs(0.8, 0.8, Units.degreesToRadians(30.0))
            .multiTagStdDevs(0.25, 0.25, Units.degreesToRadians(6.0))
            .distanceStdDevScaling(2.0, 2.0);
    public final PoseSignal photonPose = rearPhoton.pose()
            .multiTagOnCoprocessor()
            .singleTagStdDevs(0.7, 0.7, Units.degreesToRadians(25.0))
            .multiTagStdDevs(0.18, 0.18, Units.degreesToRadians(4.0))
            .distanceStdDevScaling(2.0, 2.0);
    public final PoseSignal heliosPose = driverHelios.pose()
            .singleTagStdDevs(0.9, 0.9, Units.degreesToRadians(35.0))
            .multiTagStdDevs(0.3, 0.3, Units.degreesToRadians(8.0))
            .distanceStdDevScaling(1.5, 2.0);
    public final TargetSignal limelightTargets = frontLimelight.targets();
    public final TargetSignal photonTargets = rearPhoton.targets();
    public final TargetSignal customTargets = customCamera.targets();
}
