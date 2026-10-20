package ca.frc6390.athena.mechanism.core;

import edu.wpi.first.units.measure.Current;
import ca.frc6390.athena.hardware.units.UnitConversions;
import ca.frc6390.athena.mechanism.interpolation.InterpolationData;
import ca.frc6390.athena.mechanism.interpolation.InterpolationModel;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/** A position or velocity action whose target is interpolated from runtime data. */
public final class InterpolatedControlAction implements Action, TelemetrySource {
    private static final int CURVE_SAMPLES_PER_SEGMENT = 16;
    private final ControlBinding<?> control;
    private final InterpolationModel model;
    private final DoubleSupplier input;
    private final Unit inputUnit;
    private List<Entry> entries;
    private RuntimeData runtimeData;
    private double lastInput;
    private double lastTarget;
    private boolean sampled;
    private String name = "interpolation";
    private boolean persistPoints;
    private volatile long pointVersion;
    /** The model's live-editable parameters, read by the curve fingerprint. */
    private final List<TelemetryValue> modelParameters;
    /** Last sampled curve and the fingerprint of the inputs it was computed from. Guarded by this object's lock. */
    private Translation2d[] cachedCurve;
    private long cachedCurveFingerprint;

    InterpolatedControlAction(
            ControlBinding<?> control,
            InterpolationModel model,
            DoubleSupplier input,
            Unit inputUnit) {
        this(control, model, input, inputUnit, List.of());
    }

    private InterpolatedControlAction(
            ControlBinding<?> control,
            InterpolationModel model,
            DoubleSupplier input,
            Unit inputUnit,
            List<Entry> entries) {
        this.control = Objects.requireNonNull(control, "control");
        this.model = Objects.requireNonNull(model, "model");
        this.input = Objects.requireNonNull(input, "input");
        this.inputUnit = inputUnit;
        this.entries = List.copyOf(entries);
        this.runtimeData = new RuntimeData(this.entries);
        this.modelParameters = List.copyOf(model.telemetry().values());
    }

    /**
     * Adds an interpolation point and target given as measures. The point must measure the input
     * unit passed to {@code ControlBinding.interpolate(model, input, inputUnit)}; the target must
     * measure the binding's unit (or its per-second rate for velocity bindings).
     */
    public InterpolatedControlAction at(Measure<?> point, Measure<?> value) {
        return addFixed(ControlBinding.valueIn(point, inputUnit), control.modeValue(value));
    }

    /**
     * Adds an interpolation point whose target can change at runtime. The point is a measure of the input
     * unit; the target supplier reads {@code valueUnit}, which must measure the binding's unit (per second
     * for velocity bindings).
     */
    public InterpolatedControlAction at(Measure<?> point, DoubleSupplier value, Unit valueUnit) {
        Objects.requireNonNull(value, "value");
        double factor = valueFactor(valueUnit);
        return addFixedPoint(ControlBinding.valueIn(point, inputUnit), () -> value.getAsDouble() * factor);
    }

    /**
     * Adds a runtime-sampled interpolation point and target. The point supplier reads {@code pointUnit}; the
     * target supplier reads {@code valueUnit}, which must measure the binding's unit (per second for velocity
     * bindings).
     */
    public InterpolatedControlAction at(DoubleSupplier point, Unit pointUnit, DoubleSupplier value, Unit valueUnit) {
        Objects.requireNonNull(point, "point");
        Objects.requireNonNull(value, "value");
        double pointFactor = UnitConversions.factor(Objects.requireNonNull(pointUnit, "pointUnit"), inputUnit);
        double valueFactor = valueFactor(valueUnit);
        return append(new Entry(() -> point.getAsDouble() * pointFactor,
                () -> value.getAsDouble() * valueFactor, null, null));
    }

    private double valueFactor(Unit valueUnit) {
        Objects.requireNonNull(valueUnit, "valueUnit");
        Unit target = control.unit();
        if (target == null) {
            throw new IllegalStateException("Typed interpolation values require a unit-typed control binding.");
        }
        return UnitConversions.factor(valueUnit, target);
    }

    private InterpolatedControlAction addFixed(double point, double value) {
        requireFinite(value, "Interpolation value");
        return addFixedPoint(point, () -> value);
    }

    private InterpolatedControlAction addFixedPoint(double point, DoubleSupplier value) {
        requireFinite(point, "Interpolation point");
        if (entries.stream().anyMatch(entry -> entry.fixedPoint() != null
                && Double.compare(entry.fixedPoint(), point) == 0)) {
            throw new IllegalArgumentException("Interpolation points must be unique: " + point);
        }
        return append(new Entry(() -> point, value, point, null));
    }

    private InterpolatedControlAction append(Entry entry) {
        List<Entry> updated = new ArrayList<>(entries);
        updated.add(entry);
        InterpolatedControlAction copy = new InterpolatedControlAction(control, model, input, inputUnit, updated);
        copy.name = name;
        copy.persistPoints = persistPoints;
        return copy;
    }

    /**
     * Marks this interpolation's recorded points as persistent. Points added with {@link #record()} are
     * saved in the telemetry store under the interpolation's path and restored when the robot registers.
     *
     * @return this interpolation
     */
    public InterpolatedControlAction persisted() {
        persistPoints = true;
        return this;
    }

    /**
     * Returns an action that inserts a point at the current input and target. The point is kept until
     * reboot, or across reboots when {@link #persisted()} is set. Recording plants a {@code RECORD}
     * marker with the interpolation path, input, output, and record metadata, and requests a full
     * replay capture for the current cycle. Recording the same input again replaces that point.
     *
     * @return action that records one point each time it runs
     */
    public Action record() {
        return Actions.run(this::recordCurrent);
    }

    /** Samples input and target, then inserts or replaces the recorded point. */
    synchronized void recordCurrent() {
        double target = target();
        double point = lastInput;
        RuntimeMarkers markers = RuntimeMarkers.current();
        RecordMetadata metadata = markers.recordMetadata();
        Entry recorded = new Entry(() -> point, () -> target, point, metadata);
        List<Entry> updated = new ArrayList<>(entries);
        int existing = -1;
        for (int index = 0; index < updated.size(); index++) {
            Double fixed = updated.get(index).fixedPoint();
            if (fixed != null && Double.compare(fixed, point) == 0) {
                existing = index;
                break;
            }
        }
        if (existing >= 0) {
            updated.set(existing, recorded);
        } else {
            updated.add(recorded);
        }
        replaceEntries(updated);
        pointVersion++;
        markers.mark(RuntimeMarkers.RECORD, "interpolation=" + name + " input=" + point + " output=" + target
                + " " + metadata.describe());
        markers.requestFullCapture();
    }

    /** Replaces the interpolation points and rebuilds the sampling state. */
    private void replaceEntries(List<Entry> updated) {
        entries = List.copyOf(updated);
        runtimeData = new RuntimeData(entries);
        cachedCurve = null;
    }

    /** Names this interpolation by its telemetry path. Called by the schema builder. */
    void named(String path) {
        name = Objects.requireNonNull(path, "path");
    }

    /** Returns true when recorded points persist. */
    boolean persistsPoints() {
        return persistPoints;
    }

    /** Returns a counter that changes whenever recorded points change. */
    long pointVersion() {
        return pointVersion;
    }

    /** Returns the recorded points as flattened {input, output, ...} pairs in record order. */
    synchronized double[] recordedArray() {
        int count = 0;
        for (Entry entry : entries) {
            if (entry.metadata() != null) count++;
        }
        double[] values = new double[count * 2];
        int index = 0;
        for (Entry entry : entries) {
            if (entry.metadata() == null) continue;
            values[index++] = entry.fixedPoint();
            values[index++] = entry.value().getAsDouble();
        }
        return values;
    }

    /**
     * Restores recorded points saved by a previous run. Pairs with an input already present are
     * skipped; restored points carry no match identity.
     */
    synchronized void restoreRecorded(double[] flattened) {
        if (flattened.length % 2 != 0) return;
        List<Entry> updated = new ArrayList<>(entries);
        boolean changed = false;
        for (int index = 0; index < flattened.length; index += 2) {
            double point = flattened[index];
            double value = flattened[index + 1];
            if (!Double.isFinite(point) || !Double.isFinite(value)) continue;
            boolean present = false;
            for (Entry entry : updated) {
                if (entry.fixedPoint() != null && Double.compare(entry.fixedPoint(), point) == 0) {
                    present = true;
                    break;
                }
            }
            if (present) continue;
            updated.add(new Entry(() -> point, () -> value, point, RecordMetadata.atSeconds(Double.NaN)));
            changed = true;
        }
        if (changed) {
            replaceEntries(updated);
            pointVersion++;
        }
    }

    /** Returns the control binding targeted by this action. */
    public ControlBinding<?> control() {
        return control;
    }

    /** Samples the input and interpolation values and returns the resulting target. */
    public synchronized double target() {
        if (entries.size() < 2) {
            throw new IllegalStateException("Interpolated controls require at least two points.");
        }
        double sampledInput = requireFinite(input.getAsDouble(), "Interpolation input");
        runtimeData.beginEvaluation();
        double sampledTarget = requireFinite(
                model.interpolate(sampledInput, runtimeData), "Interpolated target");
        lastInput = sampledInput;
        lastTarget = sampledTarget;
        sampled = true;
        return sampledTarget;
    }

    /**
     * Robot-loop form of {@link #target()} that never throws. A non-finite input, point, value, or
     * result, or duplicate runtime points, holds the last valid target (NaN when there is none, which
     * the runtime turns into neutral output) and reports a rate-limited warning.
     */
    synchronized double runtimeTarget() {
        try {
            return target();
        } catch (RuntimeException exception) {
            FaultIsolation.warn(this, "Interpolated " + control.mode().name().toLowerCase(java.util.Locale.ROOT)
                    + " target unavailable: " + exception.getMessage()
                    + (sampled ? "; holding last valid target " + lastTarget : "; output neutral"));
            return sampled ? lastTarget : Double.NaN;
        }
    }

    /** Returns the number of configured interpolation points. */
    public int size() {
        return entries.size();
    }

    @Override
    public Map<String, TelemetryValue> telemetry() {
        Map<String, TelemetryValue> values = new LinkedHashMap<>();
        values.put("State/Input", TelemetryValue.number(() -> sampled ? lastInput : 0.0));
        values.put("State/Output", TelemetryValue.number(() -> sampled ? lastTarget : 0.0));
        values.put("State/Sampled", TelemetryValue.bool(() -> sampled));
        values.put("Data/Inputs", TelemetryValue.numberArray(this::sampledInputs));
        values.put("Data/Values", TelemetryValue.numberArray(this::sampledValues));
        values.put("Visualization/Points", TelemetryValue.points(this::configuredPoints));
        values.put("Visualization/Curve", TelemetryValue.points(this::curvePoints));
        values.put("Visualization/Current", TelemetryValue.points(this::currentPoint));
        values.put("Recorded/Count", TelemetryValue.number(this::recordedCount));
        values.put("Recorded/Log", TelemetryValue.string(this::recordedLog));
        values.put("Config/Model", TelemetryValue.constant(model.type()));
        model.telemetry().forEach((name, value) -> values.put("Config/Curve/" + name, value));
        return Map.copyOf(values);
    }

    private synchronized double recordedCount() {
        int count = 0;
        for (Entry entry : entries) {
            if (entry.metadata() != null) count++;
        }
        return count;
    }

    private synchronized String recordedLog() {
        StringBuilder log = new StringBuilder();
        for (Entry entry : entries) {
            RecordMetadata metadata = entry.metadata();
            if (metadata == null) continue;
            if (log.length() > 0) log.append('\n');
            log.append("input=").append(entry.fixedPoint()).append(" output=")
                    .append(entry.value().getAsDouble()).append(' ').append(metadata.describe());
        }
        return log.toString();
    }

    private synchronized double[] sampledInputs() {
        if (entries.isEmpty()) return new double[0];
        try {
            runtimeData.beginEvaluation();
        } catch (RuntimeException exception) {
            return new double[0];
        }
        double[] points = new double[entries.size()];
        for (int index = 0; index < points.length; index++) points[index] = runtimeData.point(index);
        return points;
    }

    private synchronized double[] sampledValues() {
        if (entries.isEmpty()) return new double[0];
        try {
            runtimeData.beginEvaluation();
            double[] values = new double[entries.size()];
            for (int index = 0; index < values.length; index++) values[index] = runtimeData.value(index);
            return values;
        } catch (RuntimeException exception) {
            return new double[0];
        }
    }

    private synchronized Translation2d[] configuredPoints() {
        if (entries.isEmpty()) return new Translation2d[0];
        try {
            runtimeData.beginEvaluation();
            Translation2d[] points = new Translation2d[entries.size()];
            for (int index = 0; index < points.length; index++) {
                points[index] = new Translation2d(runtimeData.point(index), runtimeData.value(index));
            }
            return points;
        } catch (RuntimeException exception) {
            return new Translation2d[0];
        }
    }

    private synchronized Translation2d[] curvePoints() {
        if (entries.size() < 2) return configuredPoints();
        try {
            return sampledCurvePoints();
        } catch (RuntimeException exception) {
            return new Translation2d[0];
        }
    }

    /**
     * The curve is recomputed only when the sampled points and values, or a live model parameter, differ from the last
     * curve. A model whose output depends on state other than these is not seen until one of them changes.
     */
    private Translation2d[] sampledCurvePoints() {
        runtimeData.beginEvaluation();
        long fingerprint = curveFingerprint();
        Translation2d[] cached = cachedCurve;
        if (cached != null && fingerprint == cachedCurveFingerprint) {
            return cached;
        }
        Translation2d[] points = new Translation2d[(entries.size() - 1) * CURVE_SAMPLES_PER_SEGMENT + 1];
        int output = 0;
        for (int segment = 0; segment < entries.size() - 1; segment++) {
            double start = runtimeData.point(segment);
            double end = runtimeData.point(segment + 1);
            int firstSample = segment == 0 ? 0 : 1;
            for (int sample = firstSample; sample <= CURVE_SAMPLES_PER_SEGMENT; sample++) {
                double amount = sample / (double) CURVE_SAMPLES_PER_SEGMENT;
                double curveInput = start + (end - start) * amount;
                points[output++] = new Translation2d(
                        curveInput,
                        requireFinite(model.interpolate(curveInput, runtimeData), "Interpolated target"));
            }
        }
        cachedCurve = points;
        cachedCurveFingerprint = fingerprint;
        return points;
    }

    private long curveFingerprint() {
        long hash = 1L;
        for (int index = 0; index < entries.size(); index++) {
            hash = 31L * hash + Double.doubleToLongBits(runtimeData.point(index));
            hash = 31L * hash + Double.doubleToLongBits(runtimeData.value(index));
        }
        for (TelemetryValue parameter : modelParameters) {
            long bits = switch (parameter.type()) {
                case NUMBER -> Double.doubleToLongBits(parameter.number());
                case BOOLEAN -> parameter.bool() ? 1L : 0L;
                default -> Objects.hashCode(parameter.value());
            };
            hash = 31L * hash + bits;
        }
        return hash;
    }

    private synchronized Translation2d[] currentPoint() {
        return sampled ? new Translation2d[] {new Translation2d(lastInput, lastTarget)} : new Translation2d[0];
    }

    private static double requireFinite(double value, String description) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(description + " must be finite.");
        }
        return value;
    }

    private record Entry(DoubleSupplier point, DoubleSupplier value, Double fixedPoint, RecordMetadata metadata) {
        private Entry {
            Objects.requireNonNull(point, "point");
            Objects.requireNonNull(value, "value");
        }
    }

    private static final class RuntimeData implements InterpolationData {
        private final List<Entry> entries;
        private final double[] sampledPoints;
        private final double[] sampledValues;
        private final boolean[] sampled;
        private final int[] order;

        private RuntimeData(List<Entry> entries) {
            this.entries = entries;
            sampledPoints = new double[entries.size()];
            sampledValues = new double[entries.size()];
            sampled = new boolean[entries.size()];
            order = new int[entries.size()];
        }

        private void beginEvaluation() {
            Arrays.fill(sampled, false);
            for (int index = 0; index < entries.size(); index++) {
                sampledPoints[index] = requireFinite(
                        entries.get(index).point().getAsDouble(),
                        "Interpolation point");
                order[index] = index;
            }
            sortOrder();
            for (int index = 1; index < order.length; index++) {
                if (Double.compare(point(index - 1), point(index)) == 0) {
                    throw new IllegalArgumentException(
                            "Interpolation points must be unique: " + point(index));
                }
            }
        }

        @Override public int size() { return entries.size(); }

        @Override public double point(int index) { return sampledPoints[entryIndex(index)]; }

        @Override
        public double value(int index) {
            int entryIndex = entryIndex(index);
            if (!sampled[entryIndex]) {
                sampledValues[entryIndex] = requireFinite(
                        entries.get(entryIndex).value().getAsDouble(),
                        "Interpolation value");
                sampled[entryIndex] = true;
            }
            return sampledValues[entryIndex];
        }

        @Override
        public int lowerIndex(double input) {
            requireFinite(input, "Interpolation input");
            int insertion = insertionPoint(input);
            if (insertion < entries.size() && Double.compare(point(insertion), input) == 0) {
                return insertion;
            }
            return Math.max(0, insertion - 1);
        }

        @Override
        public int upperIndex(double input) {
            requireFinite(input, "Interpolation input");
            return Math.min(entries.size() - 1, insertionPoint(input));
        }

        private int insertionPoint(double input) {
            int low = 0;
            int high = entries.size();
            while (low < high) {
                int middle = (low + high) >>> 1;
                if (point(middle) < input) {
                    low = middle + 1;
                } else {
                    high = middle;
                }
            }
            return low;
        }

        private void sortOrder() {
            for (int index = 1; index < order.length; index++) {
                int entry = order[index];
                int insertion = index - 1;
                while (insertion >= 0
                        && sampledPoints[order[insertion]] > sampledPoints[entry]) {
                    order[insertion + 1] = order[insertion];
                    insertion--;
                }
                order[insertion + 1] = entry;
            }
        }

        private int entryIndex(int index) {
            if (index < 0 || index >= entries.size()) {
                throw new IndexOutOfBoundsException("Interpolation index " + index + " outside [0, "
                        + entries.size() + ").");
            }
            return order[index];
        }
    }
}
