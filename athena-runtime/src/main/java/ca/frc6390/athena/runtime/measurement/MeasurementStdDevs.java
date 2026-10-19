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

    /** Creates standard deviations from primitives in meters and radians. Runtime hot paths only. */
    static MeasurementStdDevs ofPrimitives(double xMeters, double yMeters, double headingRadians) {
        return new MeasurementStdDevs(xMeters, yMeters, headingRadians);
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

    /** Primitive x standard deviation in meters, for runtime measurement math. */
    double xStdDevMeters() {
        return xMeters;
    }

    /** Primitive y standard deviation in meters, for runtime measurement math. */
    double yStdDevMeters() {
        return yMeters;
    }

    /** Primitive heading standard deviation in radians, for runtime measurement math. */
    double headingStdDevRadians() {
        return headingRadians;
    }

    /**
     * Returns the average translation variance in square meters. This is an internal quantity used
     * by the localization filter; WPILib has no variance unit.
     */
    public double translationVarianceMetersSquared() {
        return (xMeters * xMeters + yMeters * yMeters) / 2.0;
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
}
