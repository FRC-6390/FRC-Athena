package ca.frc6390.athena.vision.signal;

import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.TargetMeasurementSample;
import ca.frc6390.athena.vision.device.CameraDevice;

/**
 * Target measurements produced from a camera.
 */
public interface TargetSignal extends MeasurementSignal {
    /** Returns all currently cached typed target measurements. */
    default List<TargetMeasurementSample> values() {
        return measurements().stream()
                .filter(TargetMeasurementSample.class::isInstance)
                .map(TargetMeasurementSample.class::cast)
                .toList();
    }

    /** Returns the newest currently cached target measurement. */
    default Optional<TargetMeasurementSample> latest() {
        return values().stream()
                .max(Comparator.comparingDouble(TargetMeasurementSample::timestampSeconds));
    }

    /** Returns whether at least one target measurement is available. */
    default boolean hasTarget() {
        return latest().isPresent();
    }

    /**
     * Returns the camera producing this target stream.
     *
     * @return camera
     */
    CameraDevice camera();

    /**
     * Returns target-signal metadata for graph/runtime policy decisions.
     *
     * @return metadata
     */
    default Map<String, Object> metadata() {
        return Map.of();
    }
}
