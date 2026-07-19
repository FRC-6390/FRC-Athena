package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.mechanism.interpolation.CurveMapping;
import ca.frc6390.athena.runtime.geometry.Point2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

/** Immutable processing declaration with live-tunable runtime values for one controller axis. */
public final class AxisSignal implements TelemetrySource {
    private static final int CURVE_SAMPLE_COUNT = 41;

    private final ControlOwner owner;
    private final String name;
    private final DoubleSupplier source;
    private final DoubleSupplier clock;
    private final boolean squared;
    private final boolean inverted;
    private final CurveMapping curve;
    private final boolean slewEnabled;
    private final AxisSlewLimiter slewLimiter = new AxisSlewLimiter();
    private volatile double deadband;
    private volatile double positiveSlewRate;
    private volatile double negativeSlewRate;
    private volatile double lastOutput;
    private volatile double lastInput;
    private volatile boolean sampled;
    private volatile long plotCurveFingerprint = Long.MIN_VALUE;
    private volatile Point2d[] cachedPlotCurve;

    AxisSignal(DoubleSupplier source) {
        this(null, "axis", source, Timer::getFPGATimestamp,
                0.0, false, false, AxisCurves.linear(), false, 1.0, 1.0);
    }

    AxisSignal(DoubleSupplier source, DoubleSupplier clock) {
        this(null, "axis", source, clock,
                0.0, false, false, AxisCurves.linear(), false, 1.0, 1.0);
    }

    AxisSignal(ControlOwner owner, String name, DoubleSupplier source) {
        this(owner, name, source, Timer::getFPGATimestamp,
                0.0, false, false, AxisCurves.linear(), false, 1.0, 1.0);
    }

    private AxisSignal(
            ControlOwner owner,
            String name,
            DoubleSupplier source,
            DoubleSupplier clock,
            double deadband,
            boolean squared,
            boolean inverted,
            CurveMapping curve,
            boolean slewEnabled,
            double positiveSlewRate,
            double negativeSlewRate) {
        this.owner = owner;
        this.name = requireName(name);
        this.source = Objects.requireNonNull(source, "source");
        this.clock = Objects.requireNonNull(clock, "clock");
        this.deadband = sanitizeDeadband(deadband);
        this.squared = squared;
        this.inverted = inverted;
        this.curve = Objects.requireNonNull(curve, "curve");
        this.slewEnabled = slewEnabled;
        this.positiveSlewRate = requireRate(positiveSlewRate, "positive slew rate");
        this.negativeSlewRate = requireRate(negativeSlewRate, "negative slew rate");
    }

    /** Sets the telemetry name used when this pipeline is registered with its gamepad. */
    public AxisSignal named(String name) {
        return copy(requireName(name), deadband, squared, inverted, curve,
                slewEnabled, positiveSlewRate, negativeSlewRate);
    }

    /** Applies a rescaled deadband. The value remains writable through controller telemetry. */
    public AxisSignal deadband(double deadband) {
        return copy(name, deadband, squared, inverted, curve,
                slewEnabled, positiveSlewRate, negativeSlewRate);
    }

    public AxisSignal squared() {
        return copy(name, deadband, true, inverted, curve,
                slewEnabled, positiveSlewRate, negativeSlewRate);
    }

    public AxisSignal inverted() {
        return copy(name, deadband, squared, true, curve,
                slewEnabled, positiveSlewRate, negativeSlewRate);
    }

    /** Applies a standard or custom normalized curve after deadband and squaring. */
    public AxisSignal curve(CurveMapping curve) {
        return copy(name, deadband, squared, inverted, Objects.requireNonNull(curve, "curve"),
                slewEnabled, positiveSlewRate, negativeSlewRate);
    }

    /** Applies an arbitrary mapping after deadband and squaring. */
    public AxisSignal map(DoubleUnaryOperator mapping) {
        return curve(AxisCurves.custom(mapping));
    }

    /** Limits both increasing and decreasing output by normalized units per second. */
    public AxisSignal slew(double ratePerSecond) {
        double rate = requireRate(ratePerSecond, "slew rate");
        return copy(name, deadband, squared, inverted, curve, true, rate, rate);
    }

    /** Sets separate limits for positive and negative output changes. */
    public AxisSignal slew(double positiveRatePerSecond, double negativeRatePerSecond) {
        return copy(name, deadband, squared, inverted, curve, true,
                requireRate(positiveRatePerSecond, "positive slew rate"),
                requireRate(negativeRatePerSecond, "negative slew rate"));
    }

    /** Resets the stateful slew output, primarily for mode transitions and tests. */
    public void resetSlew(double value) {
        double safeValue = clamp(value);
        slewLimiter.reset(safeValue, clock.getAsDouble());
        lastOutput = safeValue;
        sampled = true;
    }

    public DoubleSupplier toSupplier() {
        if (owner != null) owner.register(this);
        return this::value;
    }

    /** Samples the complete pipeline. */
    public double value() {
        double raw = raw();
        double mapped = mapRaw(raw);
        double output = slewEnabled
                ? slewLimiter.calculate(mapped, clock.getAsDouble(), positiveSlewRate, negativeSlewRate)
                : mapped;
        lastOutput = clamp(output);
        lastInput = raw;
        sampled = true;
        return lastOutput;
    }

    /** Returns a control signal active above the threshold. */
    public ControlSignal above(double threshold) {
        return above(threshold, threshold);
    }

    /** Returns a control signal with separate engage and release thresholds. */
    public ControlSignal above(double engage, double release) {
        requireFinite(engage, "engage");
        requireFinite(release, "release");
        if (release > engage) {
            throw new IllegalArgumentException("release must be less than or equal to engage");
        }
        return threshold("above", current -> current ? value() > release : value() > engage);
    }

    /** Returns a control signal active below the threshold. */
    public ControlSignal below(double threshold) {
        return below(threshold, threshold);
    }

    /** Returns a control signal with separate engage and release thresholds. */
    public ControlSignal below(double engage, double release) {
        requireFinite(engage, "engage");
        requireFinite(release, "release");
        if (release < engage) {
            throw new IllegalArgumentException("release must be greater than or equal to engage");
        }
        return threshold("below", current -> current ? value() < release : value() < engage);
    }

    /** Returns a control signal active outside the supplied absolute magnitude. */
    public ControlSignal outside(double magnitude) {
        requireMagnitude(magnitude);
        return new ControlSignal(owner, name + ".outside[" + magnitude + "]", context -> Math.abs(value()) > magnitude);
    }

    /** Returns a control signal active inside the supplied absolute magnitude. */
    public ControlSignal inside(double magnitude) {
        requireMagnitude(magnitude);
        return new ControlSignal(owner, name + ".inside[" + magnitude + "]", context -> Math.abs(value()) < magnitude);
    }

    @Override
    public Map<String, TelemetryValue> telemetry() {
        Map<String, TelemetryValue> values = new LinkedHashMap<>();
        values.put("State/Raw", TelemetryValue.number(this::raw));
        values.put("State/Mapped", TelemetryValue.number(() -> mapRaw(raw())));
        values.put("State/Output", TelemetryValue.number(() -> sampled ? lastOutput : mapRaw(raw())));
        values.put("Config/Deadband", TelemetryValue.writableNumber(() -> deadband, this::setDeadband));
        values.put("Config/Squared", TelemetryValue.constant(squared));
        values.put("Config/Inverted", TelemetryValue.constant(inverted));
        values.put("Curve/Type", TelemetryValue.constant(curve.type()));
        values.put("Curve/Input", TelemetryValue.constant(curveInputs()));
        values.put("Curve/Output", TelemetryValue.numberArray(this::curveOutputs));
        values.put("Visualization/Curve", TelemetryValue.points(this::plotCurve));
        values.put("Visualization/Raw", TelemetryValue.points(this::plotRaw));
        values.put("Visualization/Mapped", TelemetryValue.points(this::plotMapped));
        values.put("Visualization/Output", TelemetryValue.points(this::plotOutput));
        curve.telemetry().forEach((parameter, value) -> values.put("Curve/Config/" + parameter, value));
        values.put("Slew/Enabled", TelemetryValue.constant(slewEnabled));
        if (slewEnabled) {
            values.put("Slew/PositiveRate", TelemetryValue.writableNumber(
                    () -> positiveSlewRate, value -> positiveSlewRate = requireRate(value, "positive slew rate")));
            values.put("Slew/NegativeRate", TelemetryValue.writableNumber(
                    () -> negativeSlewRate, value -> negativeSlewRate = requireRate(value, "negative slew rate")));
        }
        return Map.copyOf(values);
    }

    String name() { return name; }

    private AxisSignal copy(
            String name,
            double deadband,
            boolean squared,
            boolean inverted,
            CurveMapping curve,
            boolean slewEnabled,
            double positiveSlewRate,
            double negativeSlewRate) {
        return new AxisSignal(owner, name, source, clock, deadband, squared, inverted, curve,
                slewEnabled, positiveSlewRate, negativeSlewRate);
    }

    private ControlSignal threshold(String operation, java.util.function.Predicate<Boolean> evaluator) {
        final class State {
            boolean active;

            boolean evaluate() {
                active = evaluator.test(active);
                return active;
            }
        }
        State state = new State();
        return new ControlSignal(owner, name + "." + operation, context -> state.evaluate());
    }

    private double raw() {
        return clamp(source.getAsDouble());
    }

    private double mapRaw(double raw) {
        double value;
        if (Math.abs(raw) <= deadband) {
            value = 0.0;
        } else {
            value = raw > 0.0
                    ? (raw - deadband) / (1.0 - deadband)
                    : (raw + deadband) / (1.0 - deadband);
        }
        if (squared) value = Math.copySign(value * value, value);
        value = curve.apply(value);
        if (!Double.isFinite(value)) value = 0.0;
        if (inverted) value = -value;
        return clamp(value);
    }

    private double[] curveOutputs() {
        double[] inputs = curveInputs();
        double[] outputs = new double[inputs.length];
        for (int index = 0; index < inputs.length; index++) {
            outputs[index] = mapRaw(inputs[index]);
        }
        return outputs;
    }

    private Point2d[] plotCurve() {
        double[] inputs = curveInputs();
        long fingerprint = 1L;
        for (double input : inputs) {
            fingerprint = 31L * fingerprint + Double.doubleToLongBits(mapRaw(input));
        }
        Point2d[] cached = cachedPlotCurve;
        if (cached != null && fingerprint == plotCurveFingerprint) return cached;

        Point2d[] points = new Point2d[inputs.length];
        for (int index = 0; index < inputs.length; index++) {
            points[index] = new Point2d(inputs[index], mapRaw(inputs[index]));
        }
        cachedPlotCurve = points;
        plotCurveFingerprint = fingerprint;
        return points;
    }

    private Point2d[] plotRaw() {
        return new Point2d[] {new Point2d(raw(), 0.0)};
    }

    private Point2d[] plotMapped() {
        double current = raw();
        return new Point2d[] {new Point2d(current, mapRaw(current))};
    }

    private Point2d[] plotOutput() {
        double input = sampled ? lastInput : raw();
        double output = sampled ? lastOutput : mapRaw(input);
        return new Point2d[] {new Point2d(input, output)};
    }

    private static double[] curveInputs() {
        double[] inputs = new double[CURVE_SAMPLE_COUNT];
        for (int index = 0; index < inputs.length; index++) {
            inputs[index] = -1.0 + 2.0 * index / (inputs.length - 1.0);
        }
        return inputs;
    }

    private void setDeadband(double requested) {
        deadband = sanitizeDeadband(requested);
    }

    private static double sanitizeDeadband(double requested) {
        if (!Double.isFinite(requested)) return 0.0;
        return Math.max(0.0, Math.min(0.99, Math.abs(requested)));
    }

    private static double requireRate(double value, String parameter) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(parameter + " must be positive and finite");
        }
        return value;
    }

    private static String requireName(String name) {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("Axis name must not be blank");
        }
        return name;
    }

    private static double clamp(double value) {
        if (!Double.isFinite(value)) return 0.0;
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private static void requireFinite(double value, String parameter) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(parameter + " must be finite");
        }
    }

    private static void requireMagnitude(double magnitude) {
        requireFinite(magnitude, "magnitude");
        if (magnitude < 0.0 || magnitude > 1.0) {
            throw new IllegalArgumentException("magnitude must be between 0 and 1");
        }
    }
}
