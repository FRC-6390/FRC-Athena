package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.TelemetryValue;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

/** Standard and custom normalized controller-axis curves. */
public final class AxisCurves {
    private AxisCurves() {
    }

    public static AxisCurve linear() {
        return new AxisCurve() {
            @Override public double apply(double input) { return input; }
            @Override public String type() { return "linear"; }
        };
    }

    /** Blends linear input with cubic input. Expo is live tunable from zero to one. */
    public static ExpoCurve expo(double expo) {
        return new ExpoCurve(expo);
    }

    /** Applies {@code sign(input) * abs(input)^exponent}. */
    public static PowerCurve power(double exponent) {
        return new PowerCurve(exponent);
    }

    /** Creates a normalized FPV-style super-rate curve with live-editable parameters. */
    public static SuperRateCurve superRate() {
        return new SuperRateCurve(1.0, 0.0, 0.0);
    }

    public static SuperRateCurve superRate(double rcRate, double superRate, double expo) {
        return new SuperRateCurve(rcRate, superRate, expo);
    }

    /** Wraps a custom mapping. Non-finite and out-of-range results are handled by {@link AxisSignal}. */
    public static AxisCurve custom(DoubleUnaryOperator mapping) {
        DoubleUnaryOperator safeMapping = Objects.requireNonNull(mapping, "mapping");
        return safeMapping::applyAsDouble;
    }

    public static final class ExpoCurve implements AxisCurve {
        private volatile double expo;

        private ExpoCurve(double expo) {
            setExpo(expo);
        }

        public ExpoCurve expo(double value) {
            setExpo(value);
            return this;
        }

        public double expo() { return expo; }

        @Override
        public double apply(double input) {
            return input * (1.0 - expo) + input * input * input * expo;
        }

        @Override public String type() { return "expo"; }

        @Override
        public Map<String, TelemetryValue> telemetry() {
            return Map.of("Expo", TelemetryValue.writableNumber(this::expo, this::setExpo));
        }

        private void setExpo(double value) {
            expo = bounded(value, 0.0, 1.0, "Expo");
        }
    }

    public static final class PowerCurve implements AxisCurve {
        private volatile double exponent;

        private PowerCurve(double exponent) {
            setExponent(exponent);
        }

        public PowerCurve exponent(double value) {
            setExponent(value);
            return this;
        }

        public double exponent() { return exponent; }

        @Override
        public double apply(double input) {
            return Math.copySign(Math.pow(Math.abs(input), exponent), input);
        }

        @Override public String type() { return "power"; }

        @Override
        public Map<String, TelemetryValue> telemetry() {
            return Map.of("Exponent", TelemetryValue.writableNumber(this::exponent, this::setExponent));
        }

        private void setExponent(double value) {
            exponent = bounded(value, 0.1, 10.0, "Curve exponent");
        }
    }

    public static final class SuperRateCurve implements AxisCurve {
        private volatile double rcRate;
        private volatile double superRate;
        private volatile double expo;

        private SuperRateCurve(double rcRate, double superRate, double expo) {
            setRcRate(rcRate);
            setSuperRate(superRate);
            setExpo(expo);
        }

        public SuperRateCurve rcRate(double value) {
            setRcRate(value);
            return this;
        }

        public SuperRateCurve superRate(double value) {
            setSuperRate(value);
            return this;
        }

        public SuperRateCurve expo(double value) {
            setExpo(value);
            return this;
        }

        public double rcRate() { return rcRate; }
        public double superRate() { return superRate; }
        public double expo() { return expo; }

        @Override
        public double apply(double input) {
            double shaped = input * (1.0 - expo) + input * input * input * expo;
            double denominator = Math.max(0.01, 1.0 - superRate * Math.abs(shaped));
            return rcRate * shaped * (1.0 - superRate) / denominator;
        }

        @Override public String type() { return "superRate"; }

        @Override
        public Map<String, TelemetryValue> telemetry() {
            Map<String, TelemetryValue> values = new LinkedHashMap<>();
            values.put("RcRate", TelemetryValue.writableNumber(this::rcRate, this::setRcRate));
            values.put("SuperRate", TelemetryValue.writableNumber(this::superRate, this::setSuperRate));
            values.put("Expo", TelemetryValue.writableNumber(this::expo, this::setExpo));
            return Map.copyOf(values);
        }

        private void setRcRate(double value) {
            rcRate = bounded(value, 0.0, 3.0, "RC rate");
        }

        private void setSuperRate(double value) {
            superRate = bounded(value, 0.0, 0.99, "Super rate");
        }

        private void setExpo(double value) {
            expo = bounded(value, 0.0, 1.0, "Expo");
        }
    }

    private static double bounded(double value, double minimum, double maximum, String name) {
        if (!Double.isFinite(value) || value < minimum || value > maximum) {
            throw new IllegalArgumentException(name + " must be between " + minimum + " and " + maximum + ".");
        }
        return value;
    }
}
