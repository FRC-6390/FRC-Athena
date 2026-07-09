package ca.frc6390.athena.runtime.measurement;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Factories for measurement signals and samples.
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
    public static Measurement pose(PoseSnapshot pose) {
        return new PoseMeasurement(pose, RobotVelocity.zero(), 0.0, 0.0, 0.0, 0, null, null);
    }

    /**
     * Creates a pose and speed measurement sample.
     *
     * @param pose pose
     * @param speeds speeds
     * @return measurement
     */
    public static Measurement poseAndSpeeds(PoseSnapshot pose, RobotVelocity speeds) {
        return new PoseMeasurement(pose, speeds, 0.0, 0.0, 0.0, 0, null, null);
    }

    /**
     * Creates a pose measurement supplier.
     *
     * @param pose pose supplier
     * @return measurement signal
     */
    public static MeasurementSignal poses(Supplier<PoseSnapshot> pose) {
        Objects.requireNonNull(pose, "pose");
        return new SingleMeasurementSignal(() -> pose(pose.get()));
    }

    /**
     * Creates a pose and speed measurement supplier.
     *
     * @param pose pose supplier
     * @param speeds speeds supplier
     * @return measurement signal
     */
    public static MeasurementSignal poseAndSpeeds(Supplier<PoseSnapshot> pose, Supplier<RobotVelocity> speeds) {
        Objects.requireNonNull(pose, "pose");
        Objects.requireNonNull(speeds, "speeds");
        return new SingleMeasurementSignal(() -> poseAndSpeeds(pose.get(), speeds.get()));
    }

    /**
     * Creates a velocity measurement supplier.
     *
     * @param speeds speeds supplier
     * @return measurement signal
     */
    public static MeasurementSignal speeds(Supplier<RobotVelocity> speeds) {
        Objects.requireNonNull(speeds, "speeds");
        return new SingleMeasurementSignal(() -> new VelocityMeasurement(speeds.get(), 0.0, 0.0, null));
    }

    /**
     * Creates a single measurement supplier.
     *
     * @param measurement measurement supplier
     * @return measurement signal
     */
    public static MeasurementSignal measurement(Supplier<? extends Measurement> measurement) {
        return new SingleMeasurementSignal(measurement);
    }

    /**
     * Creates a measurement-list supplier.
     *
     * @param measurements measurement supplier
     * @return measurement signal
     */
    public static MeasurementSignal measurements(Supplier<? extends List<? extends Measurement>> measurements) {
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
    public static Measurement custom(Object value, Map<String, Object> metadata) {
        return new CustomMeasurement(value, metadata, 0.0, 0.0, null);
    }
}
