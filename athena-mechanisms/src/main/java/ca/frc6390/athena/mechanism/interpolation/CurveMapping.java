package ca.frc6390.athena.mechanism.interpolation;

import ca.frc6390.athena.mechanism.core.TelemetryValue;
import java.util.Map;

/**
 * Reusable scalar curve shared by input mapping and shaped interpolation.
 *
 * <p>Mappings normally operate on normalized input. Interpolation normalizes each segment before
 * applying the mapping, while controller axes apply it directly to their normalized value.</p>
 */
@FunctionalInterface
public interface CurveMapping {
    /** Applies the mapping to one scalar input. */
    double apply(double input);

    /** Human-readable curve kind published with telemetry. */
    default String type() {
        return "custom";
    }

    /** Live-editable curve parameters. */
    default Map<String, TelemetryValue> telemetry() {
        return Map.of();
    }
}
