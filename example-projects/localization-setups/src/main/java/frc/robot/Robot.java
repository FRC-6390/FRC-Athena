package frc.robot;

import ca.frc6390.athena.localization.pipeline.FieldBounds;
import ca.frc6390.athena.localization.pipeline.LocalizationFilters;
import ca.frc6390.athena.localization.pipeline.LocalizationPipeline;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.device.HeliosDevice;
import ca.frc6390.athena.vision.device.LimelightDevice;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;
import ca.frc6390.athena.vision.signal.PoseSignal;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;

public final class Robot extends AthenaRobot {
    private static final MeasurementStdDevs VISION_STD_DEVS =
            MeasurementStdDevs.of(0.6, 0.6, Units.degreesToRadians(20.0));

    public final LimelightDevice frontLimelight = Cameras.limelight("limelight-front")
            .mount(new CameraMountPose(0.32, 0.18, 0.62, 0.0, -18.0, 0.0))
            .bindPose(() -> visionSample(2.0, 1.0, 0.0, 0.035, 0.08, 2));

    public final PhotonVisionDevice rearPhoton = Cameras.photonVision("photon-rear")
            .mount(new CameraMountPose(-0.28, -0.18, 0.55, 180.0, -12.0, 0.0))
            .bindPose(() -> visionSample(2.1, 1.05, 0.02, 0.045, 0.14, 3));

    public final HeliosDevice driverHelios = Cameras.helios("10.63.90.11")
            .mount(new CameraMountPose(0.0, 0.0, 0.7, 0.0, -20.0, 0.0))
            .bindPose(() -> visionSample(1.95, 1.0, -0.01, 0.06, 0.2, 1));

    public final MeasurementSignal wheelOdometry = Measurements.measurement(() ->
            Measurements.poseAndSpeeds(new PoseSnapshot(2.0, 1.0, 0.0), RobotVelocity.zero())
                    .timing(Timer.getFPGATimestamp(), 0.0));

    public final PoseSignal limelightPose = frontLimelight.pose().megatag2Blue().tags(7, 8);
    public final PoseSignal photonPose = rearPhoton.pose();
    public final PoseSignal heliosPose = driverHelios.pose();

    public final LocalizationPipeline odometryOnly = Localizations.odometry()
            .input(wheelOdometry)
            .name("odometryOnly");

    public final LocalizationPipeline filteredVision = Localizations.vision()
            .input(limelightPose, photonPose, heliosPose)
            .filter(LocalizationFilters.latencyLessThan(0.15))
            .filter(LocalizationFilters.tagCountAtLeast(2))
            .filter(FieldBounds.field(16.54, 8.21))
            .name("filteredVision");

    public final LocalizationPipeline weightedFieldPose = Localizations.weightedAverage()
            .input(odometryOnly, filteredVision)
            .filter(LocalizationFilters.maxPoseJump(odometryOnly, 1.0))
            .publishNetworkTables()
            .name("weightedFieldPose");

    public final LocalizationPipeline latestCameraPose = Localizations.latestValid()
            .input(limelightPose, photonPose)
            .filter(LocalizationFilters.latencyLessThan(0.25))
            .name("latestCameraPose");

    public final LocalizationPipeline kalmanFieldPose = Localizations.kalman()
            .input(odometryOnly, filteredVision)
            .name("kalmanFieldPose");

    @Override
    protected void configure() {
        athena()
                .cameras(frontLimelight, rearPhoton, driverHelios)
                .localization(weightedFieldPose)
                .localization(latestCameraPose)
                .localization(kalmanFieldPose)
                .localizationMaxAge(0.5);
    }

    private static List<Measurement> visionSample(
            double xMeters,
            double yMeters,
            double headingRadians,
            double latencySeconds,
            double ambiguity,
            int targetCount) {
        return List.of(Measurements.pose(new PoseSnapshot(xMeters, yMeters, headingRadians))
                .timing(Timer.getFPGATimestamp(), latencySeconds)
                .visionMetadata(ambiguity, targetCount)
                .stdDevs(VISION_STD_DEVS));
    }
}
