package ca.frc6390.athena.mechanism.interpolation;

import ca.frc6390.athena.mechanism.core.TelemetryValue;
import java.util.Map;

/** Calculates one control target from an input and ordered interpolation data. */
@FunctionalInterface
public interface InterpolationModel {
    /** Returns the interpolated target. */
    double interpolate(double input, InterpolationData data);

    /** Human-readable model kind published with interpolation telemetry. */
    default String type() {
        return "custom";
    }

    /** Live-editable parameters used by this interpolation model. */
    default Map<String, TelemetryValue> telemetry() {
        return Map.of();
    }
}
