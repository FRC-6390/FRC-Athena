package ca.frc6390.athena.sensors.camera;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Camera interface for providers that supply robot localization data.
 */
public interface LocalizationSource {
    Pose2d getLocalizationPose();

    double getLocalizationLatency();

    Matrix<N3, N1> getLocalizationStdDevs();

    VisionCamera.LocalizationData getLocalizationData();

    Optional<VisionCamera.VisionMeasurement> getLatestVisionMeasurement();
}
