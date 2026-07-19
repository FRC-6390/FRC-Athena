package ca.frc6390.athena.mechanism.interpolation;

import java.util.Objects;

/** Built-in interpolation models. */
public final class InterpolationKinds {
    /** Clamped piecewise-linear interpolation. */
    public static final InterpolationModel LINEAR = linear(CurveMappings.LINEAR);

    /**
     * Creates clamped piecewise interpolation shaped by one reusable curve mapping.
     *
     * <p>The mapping is normalized independently inside each pair of table points. Configured
     * point values therefore remain exact even when a mapping has a gain other than one.</p>
     */
    public static InterpolationModel curved(CurveMapping mapping) {
        return new MappedLinear(Objects.requireNonNull(mapping, "mapping"));
    }

    private static InterpolationModel linear(CurveMapping mapping) {
        return new MappedLinear(mapping);
    }

    private static final class MappedLinear implements InterpolationModel {
        private final CurveMapping mapping;

        private MappedLinear(CurveMapping mapping) {
            this.mapping = mapping;
        }

        @Override
        public double interpolate(double input, InterpolationData data) {
            int lower = data.lowerIndex(input);
            int upper = data.upperIndex(input);
            if (lower == upper) {
                return data.value(lower);
            }
            double lowerPoint = data.point(lower);
            double amount = (input - lowerPoint) / (data.point(upper) - lowerPoint);
            double mappedStart = finite(mapping.apply(0.0));
            double mappedEnd = finite(mapping.apply(1.0));
            double mappedRange = mappedEnd - mappedStart;
            double shaped = Math.abs(mappedRange) < 1.0e-12
                    ? amount
                    : (finite(mapping.apply(amount)) - mappedStart) / mappedRange;
            return data.value(lower) + shaped * (data.value(upper) - data.value(lower));
        }

        @Override public String type() { return mapping.type(); }

        @Override
        public java.util.Map<String, ca.frc6390.athena.mechanism.core.TelemetryValue> telemetry() {
            return mapping.telemetry();
        }
    }

    private static double finite(double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Curve mapping output must be finite.");
        }
        return value;
    }

    private InterpolationKinds() {
    }
}
