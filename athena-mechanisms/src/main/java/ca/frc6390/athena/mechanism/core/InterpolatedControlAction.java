package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.mechanism.interpolation.InterpolationData;
import ca.frc6390.athena.mechanism.interpolation.InterpolationModel;
import ca.frc6390.athena.runtime.geometry.Point2d;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/** A position or velocity action whose target is interpolated from runtime data. */
public final class InterpolatedControlAction implements Action, TelemetrySource {
    private static final int CURVE_SAMPLES_PER_SEGMENT = 16;
    private final ControlBinding control;
    private final InterpolationModel model;
    private final DoubleSupplier input;
    private final List<Entry> entries;
    private final RuntimeData runtimeData;
    private double lastInput;
    private double lastTarget;
    private boolean sampled;

    InterpolatedControlAction(ControlBinding control, InterpolationModel model, DoubleSupplier input) {
        this(control, model, input, List.of());
    }

    private InterpolatedControlAction(
            ControlBinding control,
            InterpolationModel model,
            DoubleSupplier input,
            List<Entry> entries) {
        this.control = Objects.requireNonNull(control, "control");
        this.model = Objects.requireNonNull(model, "model");
        this.input = Objects.requireNonNull(input, "input");
        this.entries = List.copyOf(entries);
        this.runtimeData = new RuntimeData(this.entries);
    }

    /** Adds a fixed interpolation point and target value. */
    public InterpolatedControlAction at(double point, double value) {
        requireFinite(value, "Interpolation value");
        return at(point, () -> value);
    }

    /** Adds an interpolation point whose target value can change at runtime. */
    public InterpolatedControlAction at(double point, DoubleSupplier value) {
        requireFinite(point, "Interpolation point");
        Objects.requireNonNull(value, "value");
        if (entries.stream().anyMatch(entry -> entry.fixedPoint() != null
                && Double.compare(entry.fixedPoint(), point) == 0)) {
            throw new IllegalArgumentException("Interpolation points must be unique: " + point);
        }
        return append(new Entry(() -> point, value, point));
    }

    /** Adds a runtime-sampled interpolation point and fixed target value. */
    public InterpolatedControlAction at(DoubleSupplier point, double value) {
        requireFinite(value, "Interpolation value");
        return at(point, () -> value);
    }

    /** Adds a runtime-sampled interpolation point and target value. */
    public InterpolatedControlAction at(DoubleSupplier point, DoubleSupplier value) {
        Objects.requireNonNull(point, "point");
        Objects.requireNonNull(value, "value");
        return append(new Entry(point, value, null));
    }

    private InterpolatedControlAction append(Entry entry) {
        List<Entry> updated = new ArrayList<>(entries);
        updated.add(entry);
        return new InterpolatedControlAction(control, model, input, updated);
    }

    /** Returns the control binding targeted by this action. */
    public ControlBinding control() {
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
        values.put("Config/Model", TelemetryValue.constant(model.type()));
        model.telemetry().forEach((name, value) -> values.put("Config/Curve/" + name, value));
        return Map.copyOf(values);
    }

    private synchronized double[] sampledInputs() {
        if (entries.isEmpty()) return new double[0];
        runtimeData.beginEvaluation();
        double[] points = new double[entries.size()];
        for (int index = 0; index < points.length; index++) points[index] = runtimeData.point(index);
        return points;
    }

    private synchronized double[] sampledValues() {
        if (entries.isEmpty()) return new double[0];
        runtimeData.beginEvaluation();
        double[] values = new double[entries.size()];
        for (int index = 0; index < values.length; index++) values[index] = runtimeData.value(index);
        return values;
    }

    private synchronized Point2d[] configuredPoints() {
        if (entries.isEmpty()) return new Point2d[0];
        runtimeData.beginEvaluation();
        Point2d[] points = new Point2d[entries.size()];
        for (int index = 0; index < points.length; index++) {
            points[index] = new Point2d(runtimeData.point(index), runtimeData.value(index));
        }
        return points;
    }

    private synchronized Point2d[] curvePoints() {
        if (entries.size() < 2) return configuredPoints();
        runtimeData.beginEvaluation();
        Point2d[] points = new Point2d[(entries.size() - 1) * CURVE_SAMPLES_PER_SEGMENT + 1];
        int output = 0;
        for (int segment = 0; segment < entries.size() - 1; segment++) {
            double start = runtimeData.point(segment);
            double end = runtimeData.point(segment + 1);
            int firstSample = segment == 0 ? 0 : 1;
            for (int sample = firstSample; sample <= CURVE_SAMPLES_PER_SEGMENT; sample++) {
                double amount = sample / (double) CURVE_SAMPLES_PER_SEGMENT;
                double curveInput = start + (end - start) * amount;
                points[output++] = new Point2d(
                        curveInput,
                        requireFinite(model.interpolate(curveInput, runtimeData), "Interpolated target"));
            }
        }
        return points;
    }

    private synchronized Point2d[] currentPoint() {
        return sampled ? new Point2d[] {new Point2d(lastInput, lastTarget)} : new Point2d[0];
    }

    private static double requireFinite(double value, String description) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(description + " must be finite.");
        }
        return value;
    }

    private record Entry(DoubleSupplier point, DoubleSupplier value, Double fixedPoint) {
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
