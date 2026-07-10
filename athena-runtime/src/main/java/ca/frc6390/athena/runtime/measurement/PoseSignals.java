package ca.frc6390.athena.runtime.measurement;

import java.util.List;
import java.util.Objects;

/** Pose-signal transformations shared by all pose-producing backends. */
public final class PoseSignals {
    private PoseSignals() {
    }

    static PoseSignal tagStdDevs(PoseSignal source, boolean multiTag, MeasurementStdDevs stdDevs) {
        Objects.requireNonNull(source, "source");
        Objects.requireNonNull(stdDevs, "stdDevs");
        return () -> source.measurements().stream()
                .map(measurement -> configureTags(measurement, multiTag, stdDevs))
                .toList();
    }

    static PoseSignal distanceStdDevScaling(PoseSignal source, double referenceMeters, double exponent) {
        Objects.requireNonNull(source, "source");
        if (!Double.isFinite(referenceMeters) || referenceMeters <= 0.0) {
            throw new IllegalArgumentException("Reference distance must be positive.");
        }
        if (!Double.isFinite(exponent) || exponent < 0.0) {
            throw new IllegalArgumentException("Distance exponent must be finite and non-negative.");
        }
        return () -> source.measurements().stream()
                .map(measurement -> scaleDistance(measurement, referenceMeters, exponent))
                .toList();
    }

    /** Returns a pose signal backed by an arbitrary measurement signal. */
    public static PoseSignal from(MeasurementSignal source) {
        Objects.requireNonNull(source, "source");
        return () -> {
            List<Measurement> values = source.measurements();
            return values == null ? List.of() : values;
        };
    }

    static Measurement withStdDevs(PoseMeasurementSample sample, MeasurementStdDevs stdDevs) {
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
}
