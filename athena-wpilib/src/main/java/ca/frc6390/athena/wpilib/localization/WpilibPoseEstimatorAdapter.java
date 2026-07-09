package ca.frc6390.athena.wpilib.localization;

import java.util.Objects;

import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Applies Athena localization vision weights to WPILib pose estimators.
 */
final class WpilibPoseEstimatorAdapter {
    private final VisionMeasurementSink sink;

    /**
     * Creates an adapter for a WPILib pose estimator.
     *
     * @param estimator WPILib pose estimator
     */
    WpilibPoseEstimatorAdapter(PoseEstimator<?> estimator) {
        this(Objects.requireNonNull(estimator, "estimator")::addVisionMeasurement);
    }

    /**
     * Creates an adapter for a custom estimator-compatible sink.
     *
     * @param sink measurement sink
     */
    WpilibPoseEstimatorAdapter(VisionMeasurementSink sink) {
        this.sink = Objects.requireNonNull(sink, "sink");
    }

    /**
     * Returns the WPILib standard-deviation vector for Athena measurement weights.
     *
     * @param stdDevs Athena standard deviations
     * @return standard deviation vector
     */
    Matrix<N3, N1> standardDeviations(MeasurementStdDevs stdDevs) {
        return standardDeviationsOrDefault(stdDevs);
    }

    /**
     * Adds a vision measurement using Athena's configured vision weights.
     *
     * @param pose measured field pose
     * @param timestampSeconds measurement timestamp
     * @param stdDevs Athena standard deviations
     */
    void addVisionMeasurement(Pose2d pose, double timestampSeconds, MeasurementStdDevs stdDevs) {
        sink.addVisionMeasurement(
                Objects.requireNonNull(pose, "pose"),
                finiteOrZero(timestampSeconds),
                standardDeviations(stdDevs));
    }

    private static Matrix<N3, N1> standardDeviationsOrDefault(MeasurementStdDevs stdDevs) {
        MeasurementStdDevs safeStdDevs = stdDevs == null
                ? MeasurementStdDevs.of(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)
                : stdDevs;
        return VecBuilder.fill(
                safeStdDevs.xMeters(),
                safeStdDevs.yMeters(),
                safeStdDevs.headingRadians());
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    /**
     * Estimator-compatible vision measurement sink.
     */
    @FunctionalInterface
    interface VisionMeasurementSink {
        /**
         * Adds a vision measurement.
         *
         * @param pose measured field pose
         * @param timestampSeconds measurement timestamp
         * @param standardDeviations WPILib standard-deviation vector
         */
        void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations);
    }
}
