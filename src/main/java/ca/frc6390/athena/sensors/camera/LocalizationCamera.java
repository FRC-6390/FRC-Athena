package ca.frc6390.athena.sensors.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface LocalizationCamera {

    boolean hasValidTarget();

    default LocalizationData getLocalizationData() {
        return new LocalizationData(getLocalizationPose(), getLocalizationLatency(), getLocalizationStdDevs());
    }

    Pose2d getLocalizationPose();
    
    double getLocalizationLatency();

    void setRobotOrientation(Double[] orientation);

    Matrix<N3, N1> getLocalizationStdDevs();

    void setStdDevs(Matrix<N3, N1> single, Matrix<N3, N1> multi);


    public static record LocalizationData(Pose2d pose, double latency, Matrix<N3, N1> stdDevs) {

    }
}
