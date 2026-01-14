package ca.frc6390.athena.logging;

interface TelemetrySink {
    TelemetryOutput create(String key, TelemetryValueType type, Object initialValue);
}
