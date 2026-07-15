package ca.frc6390.athena.mechanism.core;

import java.util.Map;

/** A mechanism declaration that contributes named telemetry or live-tuning values. */
public interface TelemetrySource {
    Map<String, TelemetryValue> telemetry();
}
