package ca.frc6390.athena.runtime.measurement;

import java.util.List;
import java.util.Objects;
import java.util.function.Function;

/** Pose-signal transformations shared by all pose-producing backends. */
public final class PoseSignals {
    private static final double UNTRUSTED_HEADING_STD_DEV_RADIANS = 1.0e6;

    private PoseSignals() {
    }

    static PoseSignal tagStdDevs(PoseSignal source, boolean multiTag, MeasurementStdDevs stdDevs) {
        Objects.requireNonNull(source, "source");
        Objects.requireNonNull(stdDevs, "stdDevs");
        return derived(source, measurements -> measurements.stream()
                .map(measurement -> configureTags(measurement, multiTag, stdDevs))
                .toList());
    }

    static PoseSignal distanceStdDevScaling(PoseSignal source, double referenceMeters, double exponent) {
        Objects.requireNonNull(source, "source");
        if (!Double.isFinite(referenceMeters) || referenceMeters <= 0.0) {
            throw new IllegalArgumentException("Reference distance must be positive.");
        }
        if (!Double.isFinite(exponent) || exponent < 0.0) {
            throw new IllegalArgumentException("Distance exponent must be finite and non-negative.");
        }
        return derived(source, measurements -> measurements.stream()
                .map(measurement -> scaleDistance(measurement, referenceMeters, exponent))
                .toList());
    }

    static PoseSignal translationOnly(PoseSignal source) {
        Objects.requireNonNull(source, "source");
        return derived(source, measurements -> measurements.stream()
                .map(PoseSignals::withoutHeading)
                .toList());
    }

    /** Returns a pose signal backed by an arbitrary measurement signal. */
    public static PoseSignal from(MeasurementSignal source) {
        Objects.requireNonNull(source, "source");
        return derived(source, values -> {
            return values == null ? List.of() : values;
        });
    }

    private static PoseSignal derived(
            MeasurementSignal source,
            Function<List<Measurement>, List<Measurement>> transform) {
        return new PoseSignal() {
            @Override
            public List<Measurement> measurements() {
                return transform.apply(source.measurements());
            }

            @Override
            public List<MeasurementSignal> sources() {
                return List.of(source);
            }
        };
    }

    static Measurement withStdDevs(PoseMeasurementSample sample, MeasurementStdDevs stdDevs) {
        if (sample instanceof TranslationOnlyPoseMeasurement) {
            return new TranslationOnlyPoseMeasurement(sample, MeasurementStdDevs.of(
                    stdDevs.xMeters(),
                    stdDevs.yMeters(),
                    UNTRUSTED_HEADING_STD_DEV_RADIANS));
        }
        return new ConfiguredPoseMeasurement(sample, stdDevs);
    }

    private static Measurement configureTags(
            Measurement measurement,
            boolean multiTag,
            MeasurementStdDevs stdDevs) {
        if (!(measurement instanceof PoseMeasurementSample sample)) {
            return measurement;
        }
        boolean sampleIsMultiTag = sample.targetCount() > 1;
        return sampleIsMultiTag == multiTag ? withStdDevs(sample, stdDevs) : measurement;
    }

    private static Measurement scaleDistance(Measurement measurement, double referenceMeters, double exponent) {
        if (!(measurement instanceof PoseMeasurementSample sample)) {
            return measurement;
        }
        double distance = sample.averageTargetDistanceMeters();
        if (!Double.isFinite(distance) || distance < 0.0) {
            return measurement;
        }
        MeasurementStdDevs stdDevs = sample.stdDevs();
        if (stdDevs == null) {
            return measurement;
        }
        double scale = Math.max(1.0, Math.pow(distance / referenceMeters, exponent));
        return withStdDevs(sample, MeasurementStdDevs.of(
                stdDevs.xMeters() * scale,
                stdDevs.yMeters() * scale,
                stdDevs.headingRadians() * scale));
    }

    private static Measurement withoutHeading(Measurement measurement) {
        if (!(measurement instanceof PoseMeasurementSample sample)) {
            return measurement;
        }
        MeasurementStdDevs configured = sample.stdDevs();
        MeasurementStdDevs stdDevs = configured == null
                ? MeasurementStdDevs.of(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
                        UNTRUSTED_HEADING_STD_DEV_RADIANS)
                : MeasurementStdDevs.of(
                        configured.xMeters(),
                        configured.yMeters(),
                        UNTRUSTED_HEADING_STD_DEV_RADIANS);
        return new TranslationOnlyPoseMeasurement(sample, stdDevs);
    }

    private record ConfiguredPoseMeasurement(
            PoseMeasurementSample delegate,
            MeasurementStdDevs stdDevs) implements PoseMeasurementSample {
        private ConfiguredPoseMeasurement {
            Objects.requireNonNull(delegate, "delegate");
            Objects.requireNonNull(stdDevs, "stdDevs");
        }

        @Override public ca.frc6390.athena.runtime.filter.PoseSnapshot pose() { return delegate.pose(); }
        @Override public ca.frc6390.athena.runtime.control.RobotVelocity speeds() { return delegate.speeds(); }
        @Override public double timestampSeconds() { return delegate.timestampSeconds(); }
        @Override public double latencySeconds() { return delegate.latencySeconds(); }
        @Override public double ambiguity() { return delegate.ambiguity(); }
        @Override public int targetCount() { return delegate.targetCount(); }
        @Override public double averageTargetDistanceMeters() { return delegate.averageTargetDistanceMeters(); }
        @Override public Object source() { return delegate.source(); }
    }

    private record TranslationOnlyPoseMeasurement(
            PoseMeasurementSample delegate,
            MeasurementStdDevs stdDevs) implements PoseMeasurementSample {
        private TranslationOnlyPoseMeasurement {
            Objects.requireNonNull(delegate, "delegate");
            Objects.requireNonNull(stdDevs, "stdDevs");
        }

        @Override public ca.frc6390.athena.runtime.filter.PoseSnapshot pose() { return delegate.pose(); }
        @Override public ca.frc6390.athena.runtime.control.RobotVelocity speeds() { return delegate.speeds(); }
        @Override public double timestampSeconds() { return delegate.timestampSeconds(); }
        @Override public double latencySeconds() { return delegate.latencySeconds(); }
        @Override public double ambiguity() { return delegate.ambiguity(); }
        @Override public int targetCount() { return delegate.targetCount(); }
        @Override public double averageTargetDistanceMeters() { return delegate.averageTargetDistanceMeters(); }
        @Override public Object source() { return delegate.source(); }
    }
}
