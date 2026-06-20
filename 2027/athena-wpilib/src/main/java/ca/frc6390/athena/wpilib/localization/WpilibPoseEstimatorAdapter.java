package ca.frc6390.athena.wpilib.localization;

import ca.frc6390.athena.localization.spec.LocalizationSpec;
import ca.frc6390.athena.localization.spec.VisionWeightSpec;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Objects;

/**
 * Applies Athena localization vision weights to WPILib pose estimators.
 */
public final class WpilibPoseEstimatorAdapter {
    private final LocalizationSpec spec;
    private final VisionMeasurementSink sink;

    /**
     * Creates an adapter for a WPILib pose estimator.
     *
     * @param spec localization spec
     * @param estimator WPILib pose estimator
     */
    public WpilibPoseEstimatorAdapter(LocalizationSpec spec, PoseEstimator<?> estimator) {
        this(spec, Objects.requireNonNull(estimator, "estimator")::addVisionMeasurement);
    }

    /**
     * Creates an adapter for a custom estimator-compatible sink.
     *
     * @param spec localization spec
     * @param sink measurement sink
     */
    public WpilibPoseEstimatorAdapter(LocalizationSpec spec, VisionMeasurementSink sink) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.sink = Objects.requireNonNull(sink, "sink");
    }

    /**
     * Returns the WPILib standard-deviation vector for the provided tag count.
     *
     * @param tagCount number of tags in the measurement
     * @return standard deviation vector
     */
    public Matrix<N3, N1> standardDeviations(int tagCount) {
        return standardDeviations(spec.visionWeight().forTagCount(tagCount));
    }

    /**
     * Adds a vision measurement using Athena's configured vision weights.
     *
     * @param pose measured field pose
     * @param timestampSeconds measurement timestamp
     * @param tagCount number of tags in the measurement
     */
    public void addVisionMeasurement(Pose2d pose, double timestampSeconds, int tagCount) {
        sink.addVisionMeasurement(
                Objects.requireNonNull(pose, "pose"),
                finiteOrZero(timestampSeconds),
                standardDeviations(tagCount));
    }

    private static Matrix<N3, N1> standardDeviations(VisionWeightSpec weight) {
        return VecBuilder.fill(
                weight.xStdDevMeters(),
                weight.yStdDevMeters(),
                weight.headingStdDevRadians());
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    /**
     * Estimator-compatible vision measurement sink.
     */
    @FunctionalInterface
    public interface VisionMeasurementSink {
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
