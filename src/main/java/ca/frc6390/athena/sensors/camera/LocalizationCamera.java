package ca.frc6390.athena.sensors.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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

    void setRobotOrientation(Pose2d pose);

    Matrix<N3, N1> getLocalizationStdDevs();

    void setStdDevs(Matrix<N3, N1> single, Matrix<N3, N1> multi);

    Matrix<N3, N1> getSingleStdDev();

    Matrix<N3, N1> getMultiStdDev();

    default Matrix<N3, N1> recalculateStdDevs(int numTags, double avgDist, double trustDistance){
        if (numTags == 0) return getSingleStdDev();

        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) return getMultiStdDev();
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > trustDistance) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        return getSingleStdDev().times(1 + (avgDist * avgDist / 30));   
    }

    public static record LocalizationData(Pose2d pose, double latency, Matrix<N3, N1> stdDevs) {

    }
}
