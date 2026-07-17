package frc.robot.examples;

import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.runtime.geometry.Circle2d;
import ca.frc6390.athena.runtime.geometry.Point2d;
import ca.frc6390.athena.runtime.geometry.Polygon2d;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import java.util.Map;

/** Annotation, custom-value, live-tuning, and field-geometry telemetry examples. */
public final class FieldTelemetry implements Mechanism, TelemetrySource {
    @Telemetry(value = "enabled", writable = true)
    public boolean enabled = true;

    @Telemetry(value = "maximumRangeMeters", writable = true, min = 1.0, max = 10.0)
    public double maximumRangeMeters = 6.0;

    @Telemetry("status")
    public String status() {
        return enabled ? "ready" : "disabled";
    }

    @Override
    public Map<String, TelemetryValue> telemetry() {
        return Map.of(
                "field/robotKeepout", TelemetryValue.geometry(Circle2d.of(4.0, 4.0, 0.75)),
                "field/loadingZone", TelemetryValue.geometry(new Rectangle2d(0.0, 0.0, 2.0, 3.0)),
                "field/scoringZone", TelemetryValue.geometry(Polygon2d.of(
                        new Point2d(13.0, 2.0),
                        new Point2d(16.0, 1.0),
                        new Point2d(16.0, 7.0),
                        new Point2d(13.0, 6.0))),
                "computed/rangeSquared", TelemetryValue.number(() -> maximumRangeMeters * maximumRangeMeters));
    }
}
