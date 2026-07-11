package ca.frc6390.athena.runtime.measurement;

import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Decorates a measurement signal with common measurement policies.
 */
final class ConfiguredMeasurementSignal implements MeasurementSignal {
    private final MeasurementSignal delegate;
    private final MeasurementStdDevs stdDevs;
    private final double latencyLimitSeconds;
    private final double maxAgeSeconds;
    private final BooleanSupplier enabled;
    private final String debugName;
    private final double nowSeconds;

    ConfiguredMeasurementSignal(
            MeasurementSignal delegate,
            MeasurementStdDevs stdDevs,
            double latencyLimitSeconds,
            double maxAgeSeconds,
            BooleanSupplier enabled,
            String debugName) {
        this(delegate, stdDevs, latencyLimitSeconds, maxAgeSeconds, enabled, debugName, 0.0);
    }

    ConfiguredMeasurementSignal(
            MeasurementSignal delegate,
            MeasurementStdDevs stdDevs,
            double latencyLimitSeconds,
            double maxAgeSeconds,
            BooleanSupplier enabled,
            String debugName,
            double nowSeconds) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.stdDevs = stdDevs;
        this.latencyLimitSeconds = sanitizeLimit(latencyLimitSeconds);
        this.maxAgeSeconds = sanitizeLimit(maxAgeSeconds);
        this.enabled = enabled;
        this.debugName = debugName == null ? "" : debugName;
        this.nowSeconds = Double.isFinite(nowSeconds) ? nowSeconds : 0.0;
    }

    @Override
    public List<Measurement> measurements() {
        if (enabled != null && !enabled.getAsBoolean()) {
            return List.of();
        }
        return delegate.measurements().stream()
                .filter(measurement -> measurement.latencySeconds() <= latencyLimitSeconds)
                .filter(measurement -> maxAgeSeconds == Double.POSITIVE_INFINITY
                        || nowSeconds - measurement.timestampSeconds() <= maxAgeSeconds)
                .map(this::applyStdDevs)
                .toList();
    }

    @Override
    public List<MeasurementSignal> sources() {
        return List.of(delegate);
    }

    @Override
    public MeasurementSignal stdDevs(MeasurementStdDevs stdDevs) {
        return new ConfiguredMeasurementSignal(delegate, stdDevs, latencyLimitSeconds, maxAgeSeconds, enabled, debugName,
                nowSeconds);
    }

    @Override
    public MeasurementSignal latencyLimit(double seconds) {
        return new ConfiguredMeasurementSignal(delegate, stdDevs, seconds, maxAgeSeconds, enabled, debugName,
                nowSeconds);
    }

    @Override
    public MeasurementSignal maxAge(double nowSeconds, double seconds) {
        return new ConfiguredMeasurementSignal(delegate, stdDevs, latencyLimitSeconds, seconds, enabled, debugName,
                nowSeconds);
    }

    @Override
    public MeasurementSignal enabled(BooleanSupplier enabled) {
        return new ConfiguredMeasurementSignal(delegate, stdDevs, latencyLimitSeconds, maxAgeSeconds, enabled, debugName,
                nowSeconds);
    }

    @Override
    public MeasurementSignal name(String name) {
        return new ConfiguredMeasurementSignal(delegate, stdDevs, latencyLimitSeconds, maxAgeSeconds, enabled, name,
                nowSeconds);
    }

    String debugName() {
        return debugName;
    }

    private Measurement applyStdDevs(Measurement measurement) {
        if (stdDevs != null && measurement instanceof PoseMeasurementSample poseMeasurement) {
            return PoseSignals.withStdDevs(poseMeasurement, stdDevs);
        }
        return measurement;
    }

    private static double sanitizeLimit(double value) {
        return Double.isFinite(value) ? Math.max(0.0, value) : Double.POSITIVE_INFINITY;
    }
}
