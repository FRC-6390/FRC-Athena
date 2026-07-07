package ca.frc6390.athena.localization.ref;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurement;

/**
 * Output from a localization node.
 *
 * @param pose estimated pose
 * @param speeds estimated speeds
 * @param timestampSeconds estimate timestamp
 * @param acceptedMeasurements accepted measurements
 * @param rejectedMeasurements rejected measurements
 */
public record LocalizationResult(
        PoseSnapshot pose,
        RobotVelocity speeds,
        double timestampSeconds,
        List<Measurement> acceptedMeasurements,
        List<Measurement> rejectedMeasurements) {
    public LocalizationResult {
        Objects.requireNonNull(pose, "pose");
        speeds = speeds == null ? RobotVelocity.zero() : speeds;
        timestampSeconds = Double.isFinite(timestampSeconds) ? timestampSeconds : 0.0;
        acceptedMeasurements = acceptedMeasurements == null ? List.of() : List.copyOf(acceptedMeasurements);
        rejectedMeasurements = rejectedMeasurements == null ? List.of() : List.copyOf(rejectedMeasurements);
    }

    /**
     * Creates a result from a pose.
     *
     * @param pose pose
     * @return result
     */
    public static LocalizationResult of(PoseSnapshot pose) {
        return new LocalizationResult(pose, RobotVelocity.zero(), 0.0, List.of(), List.of());
    }

    /**
     * Creates a result from a measurement.
     *
     * @param measurement measurement
     * @return result
     */
    public static LocalizationResult from(PoseMeasurement measurement) {
        Objects.requireNonNull(measurement, "measurement");
        return new LocalizationResult(
                measurement.pose(),
                measurement.speeds(),
                measurement.timestampSeconds(),
                List.of(measurement),
                List.of());
    }

    /**
     * Returns a copy with rejected measurements appended.
     *
     * @param rejected rejected measurements
     * @return updated result
     */
    public LocalizationResult withRejected(List<Measurement> rejected) {
        return new LocalizationResult(pose, speeds, timestampSeconds, acceptedMeasurements, rejected);
    }
}
