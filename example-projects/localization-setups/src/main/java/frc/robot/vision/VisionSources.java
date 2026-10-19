package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.device.HeliosDevice;
import ca.frc6390.athena.vision.device.LimelightDevice;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;
import ca.frc6390.athena.vision.signal.TargetSignal;
import java.util.List;

public final class VisionSources {
    public final LimelightDevice frontLimelight = Cameras.limelight("limelight-front")
            .mount(new CameraMountPose(Meters.of(0.32), Meters.of(0.18), Meters.of(0.62), Degrees.of(0.0), Degrees.of(-18.0), Degrees.of(0.0)));
    public final PhotonVisionDevice rearPhoton = Cameras.photonVision("photon-rear")
            .mount(new CameraMountPose(Meters.of(-0.28), Meters.of(-0.18), Meters.of(0.55), Degrees.of(180.0), Degrees.of(-12.0), Degrees.of(0.0)));
    public final HeliosDevice driverHelios = Cameras.helios("10.63.90.11")
            .mount(new CameraMountPose(Meters.of(0.0), Meters.of(0.0), Meters.of(0.7), Degrees.of(0.0), Degrees.of(-20.0), Degrees.of(0.0)));
    public final CameraDevice customCamera = Cameras.camera(
            () -> "team:target-camera", "custom-target-camera")
            .bindTargets(() -> List.of(Measurements.target(42, Degrees.of(3.5), Degrees.of(-8.0), Meters.of(2.7), 0.95)));

    public final PoseSignal limelightPose = frontLimelight.pose()
            .megatag2Blue()
            .tags(7, 8)
            .singleTagStdDevs(Meters.of(0.8), Meters.of(0.8), Degrees.of(30.0))
            .multiTagStdDevs(Meters.of(0.25), Meters.of(0.25), Degrees.of(6.0))
            .distanceStdDevScaling(Meters.of(2.0), 2.0);
    // Single-tag frames are solved with the robot heading (needs PhotonVisionCameraAdapter.defaultRobotHeading).
    public final PoseSignal photonPose = rearPhoton.pose()
            .multiTagOnCoprocessor().singleTagUsesHeading()
            .singleTagStdDevs(Meters.of(0.7), Meters.of(0.7), Degrees.of(25.0))
            .multiTagStdDevs(Meters.of(0.18), Meters.of(0.18), Degrees.of(4.0))
            .distanceStdDevScaling(Meters.of(2.0), 2.0);
    public final PoseSignal heliosPose = driverHelios.pose()
            .singleTagStdDevs(Meters.of(0.9), Meters.of(0.9), Degrees.of(35.0))
            .multiTagStdDevs(Meters.of(0.3), Meters.of(0.3), Degrees.of(8.0))
            .distanceStdDevScaling(Meters.of(1.5), 2.0);
    public final TargetSignal limelightTargets = frontLimelight.targets();
    public final TargetSignal photonTargets = rearPhoton.targets();
    public final TargetSignal customTargets = customCamera.targets();
}
