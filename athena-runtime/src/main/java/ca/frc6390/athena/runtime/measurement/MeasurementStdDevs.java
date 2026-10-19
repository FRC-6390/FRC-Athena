package ca.frc6390.athena.runtime.measurement;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.Objects;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

/**
 * Standard deviations for a field-relative pose measurement. Stored as primitives (meters and
 * radians) so that per-frame measurement creation does not allocate measures.
 */
public final class MeasurementStdDevs {
    private final double xMeters;
    private final double yMeters;
    private final double headingRadians;

    private MeasurementStdDevs(double xMeters, double yMeters, double headingRadians) {
        this.xMeters = positiveOrInfinite(xMeters);
        this.yMeters = positiveOrInfinite(yMeters);
        this.headingRadians = positiveOrInfinite(headingRadians);
    }

    /**
     * Creates pose standard deviations.
     *
     * @param xStdDev x standard deviation
     * @param yStdDev y standard deviation
     * @param headingStdDev heading standard deviation
     * @return standard deviations
     */
    public static MeasurementStdDevs of(Distance xStdDev, Distance yStdDev, Angle headingStdDev) {
        return new MeasurementStdDevs(
                Objects.requireNonNull(xStdDev, "xStdDev").in(Meters),
                Objects.requireNonNull(yStdDev, "yStdDev").in(Meters),
                Objects.requireNonNull(headingStdDev, "headingStdDev").in(Radians));
    }

    /** Returns the x standard deviation. */
    public Distance xStdDev() {
        return Meters.of(xMeters);
    }

    /** Returns the y standard deviation. */
    public Distance yStdDev() {
        return Meters.of(yMeters);
    }

    /** Returns the heading standard deviation. */
    public Angle headingStdDev() {
        return Radians.of(headingRadians);
    }

    /** Returns an internal, primitive view of these standard deviations. Not for robot code. */
    public Raw raw() {
        return new Raw(this);
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof MeasurementStdDevs other
                && Double.compare(xMeters, other.xMeters) == 0
                && Double.compare(yMeters, other.yMeters) == 0
                && Double.compare(headingRadians, other.headingRadians) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(xMeters, yMeters, headingRadians);
    }

    @Override
    public String toString() {
        return "MeasurementStdDevs[x=" + xMeters + " m, y=" + yMeters + " m, heading=" + headingRadians + " rad]";
    }

    private static double positiveOrInfinite(double value) {
        if (!Double.isFinite(value)) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(1.0e-9, value);
    }

    /**
     * Internal, not for robot code: primitive access for runtime and localization math. Units are
     * meters and radians.
     */
    public static final class Raw {
        private final MeasurementStdDevs stdDevs;

        private Raw(MeasurementStdDevs stdDevs) {
            this.stdDevs = stdDevs;
        }

        /** Internal: builds standard deviations from primitives without allocating measures. */
        public static MeasurementStdDevs of(double xMeters, double yMeters, double headingRadians) {
            return new MeasurementStdDevs(xMeters, yMeters, headingRadians);
        }

        /** Internal: x standard deviation in meters. */
        public double xMeters() {
            return stdDevs.xMeters;
        }

        /** Internal: y standard deviation in meters. */
        public double yMeters() {
            return stdDevs.yMeters;
        }

        /** Internal: heading standard deviation in radians. */
        public double headingRadians() {
            return stdDevs.headingRadians;
        }

        /** Internal: average translation variance in square meters. */
        public double translationVarianceMetersSquared() {
            return (stdDevs.xMeters * stdDevs.xMeters + stdDevs.yMeters * stdDevs.yMeters) / 2.0;
        }
    }
}
