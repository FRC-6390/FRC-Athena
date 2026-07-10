package ca.frc6390.athena.runtime.measurement;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Typed pose-capable measurement sample.
 */
public interface PoseMeasurementSample extends Measurement {
    /**
     * Returns the measured field-relative pose.
     *
     * @return pose
     */
    PoseSnapshot pose();

    /**
     * Returns the measured robot velocity.
     *
     * @return robot velocity
     */
    RobotVelocity speeds();

    /**
     * Returns source ambiguity where lower is better.
     *
     * @return ambiguity
     */
    double ambiguity();

    /**
     * Returns the number of targets used by this pose estimate.
     *
     * @return target count
     */
    int targetCount();

    /** Returns average target distance, or NaN when the source does not provide it. */
    default double averageTargetDistanceMeters() {
        return Double.NaN;
    }

    /**
     * Returns measurement standard deviations.
     *
     * @return standard deviations
     */
    MeasurementStdDevs stdDevs();

    /**
     * Returns a copy with standard deviations.
     *
     * @param stdDevs standard deviations
     * @return updated sample
     */
    default PoseMeasurementSample stdDevs(MeasurementStdDevs stdDevs) {
        return this;
    }

    /**
     * Returns a copy with timing metadata.
     *
     * @param timestampSeconds timestamp
     * @param latencySeconds latency
     * @return updated sample
     */
    default PoseMeasurementSample timing(double timestampSeconds, double latencySeconds) {
        return this;
    }

    /**
     * Returns a copy with vision-style metadata.
     *
     * @param ambiguity ambiguity
     * @param targetCount target count
     * @return updated sample
     */
    default PoseMeasurementSample visionMetadata(double ambiguity, int targetCount) {
        return visionMetadata(ambiguity, targetCount, averageTargetDistanceMeters());
    }

    /** Returns a copy with complete vision quality metadata. */
    default PoseMeasurementSample visionMetadata(
            double ambiguity,
            int targetCount,
            double averageTargetDistanceMeters) {
        return this;
    }

    /**
     * Returns a copy with source metadata.
     *
     * @param source source object
     * @return updated sample
     */
    default PoseMeasurementSample source(Object source) {
        return this;
    }
}
