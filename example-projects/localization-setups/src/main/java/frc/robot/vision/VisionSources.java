package frc.robot.vision;

import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.device.HeliosDevice;
import ca.frc6390.athena.vision.device.LimelightDevice;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;
import edu.wpi.first.math.util.Units;

public final class VisionSources {
    public final LimelightDevice frontLimelight = Cameras.limelight("limelight-front")
            .mount(new CameraMountPose(0.32, 0.18, 0.62, 0.0, -18.0, 0.0));
    public final PhotonVisionDevice rearPhoton = Cameras.photonVision("photon-rear")
            .mount(new CameraMountPose(-0.28, -0.18, 0.55, 180.0, -12.0, 0.0));
    public final HeliosDevice driverHelios = Cameras.helios("10.63.90.11")
            .mount(new CameraMountPose(0.0, 0.0, 0.7, 0.0, -20.0, 0.0));

    public final MeasurementSignal limelightPose = frontLimelight.pose()
            .megatag2Blue()
            .tags(7, 8)
            .stdDevs(0.6, Units.degreesToRadians(20.0));
    public final MeasurementSignal photonPose = rearPhoton.pose()
            .stdDevs(0.6, Units.degreesToRadians(20.0));
    public final MeasurementSignal heliosPose = driverHelios.pose()
            .stdDevs(0.6, Units.degreesToRadians(20.0));
}
