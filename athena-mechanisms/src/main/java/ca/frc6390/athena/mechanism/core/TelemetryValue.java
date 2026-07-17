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
    private final DoubleSupplier numberReader;
    private final BooleanSupplier booleanReader;
    private final java.util.function.Consumer<Object> writer;
    private final boolean momentary;
    private final boolean constant;

    private TelemetryValue(Type type, boolean writable, Supplier<Object> reader,
            java.util.function.Consumer<Object> writer) {
        this(type, writable, reader, writer, false, false);
    }

    private TelemetryValue(Type type, boolean writable, Supplier<Object> reader,
            java.util.function.Consumer<Object> writer, boolean momentary) {
        this(type, writable, reader, writer, momentary, false);
    }

    private TelemetryValue(Type type, boolean writable, Supplier<Object> reader,
            java.util.function.Consumer<Object> writer, boolean momentary, boolean constant) {
        this.type = Objects.requireNonNull(type, "type");
        this.writable = writable;
        this.reader = Objects.requireNonNull(reader, "reader");
        this.numberReader = null;
        this.booleanReader = null;
        this.writer = writer;
        this.momentary = momentary;
        this.constant = constant;
    }

    private TelemetryValue(
            Type type,
            boolean writable,
            DoubleSupplier numberReader,
            BooleanSupplier booleanReader,
            java.util.function.Consumer<Object> writer) {
        this(type, writable, numberReader, booleanReader, writer, false);
    }

    private TelemetryValue(
            Type type,
            boolean writable,
            DoubleSupplier numberReader,
            BooleanSupplier booleanReader,
            java.util.function.Consumer<Object> writer,
            boolean constant) {
        this.type = Objects.requireNonNull(type, "type");
        this.writable = writable;
        this.reader = null;
        this.numberReader = numberReader;
        this.booleanReader = booleanReader;
        this.writer = writer;
        this.momentary = false;
        this.constant = constant;
    }

    /** Creates an immutable numeric value captured once when the schema is built. */
    public static TelemetryValue constant(double value) {
        return new TelemetryValue(Type.NUMBER, false, () -> value, null, null, true);
    }

    /** Creates an immutable boolean value captured once when the schema is built. */
    public static TelemetryValue constant(boolean value) {
        return new TelemetryValue(Type.BOOLEAN, false, null, () -> value, null, true);
    }

    /** Creates an immutable string value captured once when the schema is built. */
    public static TelemetryValue constant(String value) {
        String captured = Objects.requireNonNull(value, "value");
        return new TelemetryValue(Type.STRING, false, () -> captured, null, false, true);
    }

    /** Creates immutable field geometry captured once when the schema is built. */
    public static TelemetryValue constant(Geometry2d value) {
        Geometry2d captured = Objects.requireNonNull(value, "value");
        return new TelemetryValue(Type.GEOMETRY, false, () -> captured, null, false, true);
    }

    public static TelemetryValue number(DoubleSupplier value) {
        Objects.requireNonNull(value, "value");
        return new TelemetryValue(Type.NUMBER, false, value, null, null);
    }
    public static TelemetryValue bool(BooleanSupplier value) {
        Objects.requireNonNull(value, "value");
        return new TelemetryValue(Type.BOOLEAN, false, null, value, null);
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
        return new TelemetryValue(Type.NUMBER, true, value::get, null,
                raw -> value.set(((Number) raw).doubleValue()));
    }

    /** Creates a writable boolean value for an application-specific mechanism setting. */
    public static TelemetryValue bool(boolean initialValue) {
        MutableBoolean value = new MutableBoolean(initialValue);
        return new TelemetryValue(Type.BOOLEAN, true, null, value::get,
                raw -> value.set((Boolean) raw));
    }

    public static TelemetryValue writableNumber(DoubleSupplier reader, DoubleConsumer writer) {
        DoubleConsumer safeWriter = Objects.requireNonNull(writer, "writer");
        return new TelemetryValue(Type.NUMBER, true,
                Objects.requireNonNull(reader, "reader"), null,
                raw -> safeWriter.accept(((Number) raw).doubleValue()));
    }

    public static TelemetryValue writableBoolean(
            BooleanSupplier reader, java.util.function.Consumer<Boolean> writer) {
        java.util.function.Consumer<Boolean> safeWriter = Objects.requireNonNull(writer, "writer");
        return new TelemetryValue(Type.BOOLEAN, true, null,
                Objects.requireNonNull(reader, "reader"),
                raw -> safeWriter.accept((Boolean) raw));
    }
    public static TelemetryValue writableString(
            Supplier<String> reader, java.util.function.Consumer<String> writer) {
        java.util.function.Consumer<String> safeWriter = Objects.requireNonNull(writer, "writer");
        return new TelemetryValue(Type.STRING, true,
                Objects.requireNonNull(reader, "reader")::get,
                raw -> safeWriter.accept(String.valueOf(raw)));
    }

    /** Creates a momentary boolean operation; writing true invokes the operation. */
    public static TelemetryValue command(Runnable operation) {
        Runnable safeOperation = Objects.requireNonNull(operation, "operation");
        return new TelemetryValue(Type.BOOLEAN, true, () -> false, raw -> {
            if (Boolean.TRUE.equals(raw)) safeOperation.run();
        }, true);
    }
    public Type type() { return type; }
    public boolean writable() { return writable; }
    public boolean momentary() { return momentary; }
    /** True when the value never changes after schema construction. */
    public boolean constant() { return constant; }
    public Object value() {
        if (numberReader != null) return numberReader.getAsDouble();
        if (booleanReader != null) return booleanReader.getAsBoolean();
        return reader.get();
    }
    public double number() {
        if (type != Type.NUMBER) throw new IllegalStateException("Telemetry value is not numeric.");
        return numberReader != null ? numberReader.getAsDouble() : ((Number) reader.get()).doubleValue();
    }
    public boolean bool() {
        if (type != Type.BOOLEAN) throw new IllegalStateException("Telemetry value is not boolean.");
        return booleanReader != null ? booleanReader.getAsBoolean() : (Boolean) reader.get();
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
