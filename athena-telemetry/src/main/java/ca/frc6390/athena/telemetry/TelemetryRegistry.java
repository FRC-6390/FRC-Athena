package ca.frc6390.athena.telemetry;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Registry of telemetry suppliers.
 */
public final class TelemetryRegistry {
    private final Map<TelemetryKey, Supplier<TelemetryValue>> suppliers = new LinkedHashMap<>();

    /**
     * Registers a boolean supplier.
     *
     * @param key telemetry key
     * @param supplier value supplier
     * @return this registry
     */
    public TelemetryRegistry booleanValue(TelemetryKey key, BooleanSupplier supplier) {
        requireType(key, TelemetryType.BOOLEAN);
        suppliers.put(key, () -> TelemetryValue.of(supplier.getAsBoolean()));
        return this;
    }

    /**
     * Registers a numeric supplier.
     *
     * @param key telemetry key
     * @param supplier value supplier
     * @return this registry
     */
    public TelemetryRegistry numberValue(TelemetryKey key, DoubleSupplier supplier) {
        requireType(key, TelemetryType.NUMBER);
        suppliers.put(key, () -> TelemetryValue.of(supplier.getAsDouble()));
        return this;
    }

    /**
     * Registers a string supplier.
     *
     * @param key telemetry key
     * @param supplier value supplier
     * @return this registry
     */
    public TelemetryRegistry stringValue(TelemetryKey key, Supplier<String> supplier) {
        requireType(key, TelemetryType.STRING);
        suppliers.put(key, () -> TelemetryValue.of(supplier.get()));
        return this;
    }

    /**
     * Publishes all registered values to a sink.
     *
     * @param sink telemetry sink
     */
    public void publishAll(TelemetrySink sink) {
        suppliers.forEach((key, supplier) -> sink.publish(key, supplier.get()));
    }

    /**
     * Captures current telemetry values.
     *
     * @return snapshot
     */
    public TelemetrySnapshot snapshot() {
        Map<TelemetryKey, TelemetryValue> values = new LinkedHashMap<>();
        suppliers.forEach((key, supplier) -> values.put(key, supplier.get()));
        return new TelemetrySnapshot(values);
    }

    private void requireType(TelemetryKey key, TelemetryType expected) {
        if (key.type() != expected) {
            throw new IllegalArgumentException("Telemetry key " + key.path() + " must be " + expected + ".");
        }
    }
}
