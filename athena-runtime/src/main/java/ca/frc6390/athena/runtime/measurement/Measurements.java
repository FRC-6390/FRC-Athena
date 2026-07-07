package ca.frc6390.athena.runtime.measurement;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Factories for measurement refs and samples.
 */
public final class Measurements {
    private Measurements() {
    }

    /**
     * Creates a pose measurement sample.
     *
     * @param pose pose
     * @return measurement
     */
    public static PoseMeasurement pose(PoseSnapshot pose) {
        return new PoseMeasurement(pose, RobotVelocity.zero(), 0.0, 0.0, 0.0, 0, null, null);
    }

    /**
     * Creates a pose and speed measurement sample.
     *
     * @param pose pose
     * @param speeds speeds
     * @return measurement
     */
    public static PoseMeasurement poseAndSpeeds(PoseSnapshot pose, RobotVelocity speeds) {
        return new PoseMeasurement(pose, speeds, 0.0, 0.0, 0.0, 0, null, null);
    }

    /**
     * Creates a pose measurement supplier.
     *
     * @param pose pose supplier
     * @return measurement ref
     */
    public static MeasurementRef poses(Supplier<PoseSnapshot> pose) {
        Objects.requireNonNull(pose, "pose");
        return () -> List.of(pose(pose.get()));
    }

    /**
     * Creates a pose and speed measurement supplier.
     *
     * @param pose pose supplier
     * @param speeds speeds supplier
     * @return measurement ref
     */
    public static MeasurementRef poseAndSpeeds(Supplier<PoseSnapshot> pose, Supplier<RobotVelocity> speeds) {
        Objects.requireNonNull(pose, "pose");
        Objects.requireNonNull(speeds, "speeds");
        return () -> List.of(poseAndSpeeds(pose.get(), speeds.get()));
    }

    /**
     * Creates a velocity measurement supplier.
     *
     * @param speeds speeds supplier
     * @return measurement ref
     */
    public static MeasurementRef speeds(Supplier<RobotVelocity> speeds) {
        Objects.requireNonNull(speeds, "speeds");
        return () -> List.of(new VelocityMeasurement(speeds.get(), 0.0, 0.0, null));
    }

    /**
     * Creates a single measurement supplier.
     *
     * @param measurement measurement supplier
     * @return measurement ref
     */
    public static MeasurementRef measurement(Supplier<? extends Measurement> measurement) {
        Objects.requireNonNull(measurement, "measurement");
        return () -> {
            Measurement value = measurement.get();
            return value == null ? List.of() : List.of(value);
        };
    }

    /**
     * Creates a measurement-list supplier.
     *
     * @param measurements measurement supplier
     * @return measurement ref
     */
    public static MeasurementRef measurements(Supplier<? extends List<? extends Measurement>> measurements) {
        Objects.requireNonNull(measurements, "measurements");
        return () -> {
            List<? extends Measurement> values = measurements.get();
            return values == null ? List.of() : List.copyOf(values);
        };
    }

    /**
     * Creates a custom measurement sample.
     *
     * @param value value object
     * @param metadata metadata
     * @return measurement
     */
    public static CustomMeasurement custom(Object value, Map<String, Object> metadata) {
        return new CustomMeasurement(value, metadata, 0.0, 0.0, null);
    }
}
