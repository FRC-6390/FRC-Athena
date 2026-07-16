package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.runtime.geometry.Geometry2d;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** A lightweight mechanism-owned value that may be published or tuned live. */
public final class TelemetryValue {
    public enum Type { NUMBER, BOOLEAN, STRING, GEOMETRY }
    private final Type type;
    private final boolean writable;
    private final Supplier<Object> reader;
    private final java.util.function.Consumer<Object> writer;

    private TelemetryValue(Type type, boolean writable, Supplier<Object> reader,
            java.util.function.Consumer<Object> writer) {
        this.type = Objects.requireNonNull(type, "type");
        this.writable = writable;
        this.reader = Objects.requireNonNull(reader, "reader");
        this.writer = writer;
    }

    public static TelemetryValue number(DoubleSupplier value) {
        Objects.requireNonNull(value, "value");
        return new TelemetryValue(Type.NUMBER, false, value::getAsDouble, null);
    }
    public static TelemetryValue bool(BooleanSupplier value) {
        Objects.requireNonNull(value, "value");
        return new TelemetryValue(Type.BOOLEAN, false, value::getAsBoolean, null);
    }
    public static TelemetryValue string(Supplier<String> value) {
        return new TelemetryValue(Type.STRING, false, Objects.requireNonNull(value, "value")::get, null);
    }
    /** Creates read-only field geometry telemetry for WPILib field visualization. */
    public static TelemetryValue geometry(Geometry2d value) {
        Geometry2d geometry = Objects.requireNonNull(value, "value");
        return new TelemetryValue(Type.GEOMETRY, false, () -> geometry, null);
    }
    /** Creates read-only field geometry telemetry evaluated when telemetry is sampled. */
    public static TelemetryValue dynamicGeometry(Supplier<? extends Geometry2d> value) {
        Objects.requireNonNull(value, "value");
        return new TelemetryValue(Type.GEOMETRY, false, value::get, null);
    }
    /** Creates a writable numeric value for an application-specific mechanism setting. */
    public static TelemetryValue number(double initialValue) {
        MutableDouble value = new MutableDouble(initialValue);
        return new TelemetryValue(Type.NUMBER, true, value::get, raw -> value.set(((Number) raw).doubleValue()));
    }

    /** Creates a writable boolean value for an application-specific mechanism setting. */
    public static TelemetryValue bool(boolean initialValue) {
        MutableBoolean value = new MutableBoolean(initialValue);
        return new TelemetryValue(Type.BOOLEAN, true, value::get, raw -> value.set((Boolean) raw));
    }

    public static TelemetryValue writableNumber(DoubleSupplier reader, DoubleConsumer writer) {
        return new TelemetryValue(Type.NUMBER, true,
                Objects.requireNonNull(reader, "reader")::getAsDouble,
                raw -> Objects.requireNonNull(writer, "writer").accept(((Number) raw).doubleValue()));
    }

    public static TelemetryValue writableBoolean(
            BooleanSupplier reader, java.util.function.Consumer<Boolean> writer) {
        return new TelemetryValue(Type.BOOLEAN, true,
                Objects.requireNonNull(reader, "reader")::getAsBoolean,
                raw -> Objects.requireNonNull(writer, "writer").accept((Boolean) raw));
    }
    public Type type() { return type; }
    public boolean writable() { return writable; }
    public Object value() { return reader.get(); }
    public double number() {
        if (type != Type.NUMBER) throw new IllegalStateException("Telemetry value is not numeric.");
        return ((Number) value()).doubleValue();
    }
    public boolean bool() {
        if (type != Type.BOOLEAN) throw new IllegalStateException("Telemetry value is not boolean.");
        return (Boolean) value();
    }
    public void set(Object value) {
        if (!writable || writer == null) throw new UnsupportedOperationException("Telemetry value is read-only.");
        writer.accept(value);
    }

    private static final class MutableDouble {
        private volatile double value;
        private volatile double minimum = -Double.MAX_VALUE;
        private volatile double maximum = Double.MAX_VALUE;
        private volatile DoubleConsumer onChange = ignored -> { };
        private MutableDouble(double initialValue) { set(initialValue); }
        public void set(double next) {
            if (!Double.isFinite(next)) return;
            double clamped = Math.max(minimum, Math.min(maximum, next));
            if (Double.compare(value, clamped) != 0) { value = clamped; onChange.accept(clamped); }
        }
        private double get() { return value; }
    }

    private static final class MutableBoolean {
        private volatile boolean value;
        private volatile java.util.function.Consumer<Boolean> onChange = ignored -> { };
        private MutableBoolean(boolean initialValue) { value = initialValue; }
        public void set(boolean next) {
            if (value != next) { value = next; onChange.accept(next); }
        }
        private boolean get() { return value; }
    }
}
