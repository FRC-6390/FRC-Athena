package ca.frc6390.athena.core.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public interface PoseEstimatorFactory<T> {
    PoseEstimator<T> create2d(Pose2d startPose, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionStdDevs);

    PoseEstimator3d<T> create3d(Pose3d startPose, Matrix<N4, N1> stateStdDevs, Matrix<N4, N1> visionStdDevs);
}
