package ca.frc6390.athena.sensors.camera;

import edu.wpi.first.math.geometry.Pose2d;

public interface LocalizationCamera {

    boolean hasValidTarget();

    Pose2d getLocalizationPose();
    
    double getLocalizationLatency();
}
