package ca.frc6390.athena.runtime.measurement;

import java.util.Objects;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Field-relative pose measurement.
 *
 * @param pose measured pose
 * @param speeds measured robot speeds
 * @param timestampSeconds measurement timestamp
 * @param latencySeconds measurement latency
 * @param ambiguity source ambiguity, where lower is better
 * @param targetCount number of targets used by the measurement
 * @param stdDevs standard deviations
 * @param source source object
 */
record PoseMeasurement(
        PoseSnapshot pose,
        RobotVelocity speeds,
        double timestampSeconds,
        double latencySeconds,
        double ambiguity,
        int targetCount,
        MeasurementStdDevs stdDevs,
        Object source) implements PoseMeasurementSample {
    public PoseMeasurement {
        Objects.requireNonNull(pose, "pose");
        speeds = speeds == null ? RobotVelocity.zero() : speeds;
        timestampSeconds = finiteOrZero(timestampSeconds);
        latencySeconds = Math.max(0.0, finiteOrZero(latencySeconds));
        ambiguity = Math.max(0.0, finiteOrZero(ambiguity));
        targetCount = Math.max(0, targetCount);
        stdDevs = stdDevs == null
                ? MeasurementStdDevs.of(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)
                : stdDevs;
    }

    /**
     * Returns a copy with standard deviations.
     *
     * @param stdDevs standard deviations
     * @return updated measurement
     */
    public PoseMeasurementSample stdDevs(MeasurementStdDevs stdDevs) {
        return new PoseMeasurement(pose, speeds, timestampSeconds, latencySeconds, ambiguity, targetCount, stdDevs, source);
    }

    /**
     * Returns a copy with timing metadata.
     *
     * @param timestampSeconds timestamp
     * @param latencySeconds latency
     * @return updated measurement
     */
    public PoseMeasurementSample timing(double timestampSeconds, double latencySeconds) {
        return new PoseMeasurement(pose, speeds, timestampSeconds, latencySeconds, ambiguity, targetCount, stdDevs, source);
    }

    /**
     * Returns a copy with vision-style metadata.
     *
     * @param ambiguity ambiguity
     * @param targetCount target count
     * @return updated measurement
     */
    public PoseMeasurementSample visionMetadata(double ambiguity, int targetCount) {
        return new PoseMeasurement(pose, speeds, timestampSeconds, latencySeconds, ambiguity, targetCount, stdDevs, source);
    }

    /**
     * Returns a copy with source metadata.
     *
     * @param source source object
     * @return updated measurement
     */
    public PoseMeasurementSample source(Object source) {
        return new PoseMeasurement(pose, speeds, timestampSeconds, latencySeconds, ambiguity, targetCount, stdDevs, source);
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
