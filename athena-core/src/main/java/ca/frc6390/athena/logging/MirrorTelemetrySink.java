package ca.frc6390.athena.logging;

final class MirrorTelemetrySink implements TelemetrySink {
    private final TelemetrySink primary;
    private final TelemetrySink mirror;

    MirrorTelemetrySink(TelemetrySink primary, TelemetrySink mirror) {
        this.primary = primary;
        this.mirror = mirror;
    }

    @Override
    public TelemetryOutput create(String key, TelemetryValueType type, Object initialValue) {
        TelemetryOutput primaryOutput = primary != null ? primary.create(key, type, initialValue) : null;
        TelemetryOutput mirrorOutput = mirror != null ? mirror.create(key, type, initialValue) : null;
        if (primaryOutput == null) {
            return mirrorOutput;
        }
        if (mirrorOutput == null) {
            return primaryOutput;
        }
        return value -> {
            primaryOutput.write(value);
            mirrorOutput.write(value);
        };
    }
}
