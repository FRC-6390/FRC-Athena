package ca.frc6390.athena.hardware.device;

import java.util.Objects;

import ca.frc6390.athena.hardware.units.UnitConversions;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Reusable mechanism travel range.
 *
 * <p>The bounds keep the unit they were declared in, so {@code Range.of(Degrees.of(-270), Degrees.of(270))}
 * clamps values in degrees. {@link #clamp(double)} is the per-cycle primitive used by constraints; it
 * takes and returns values in {@link #unit()} and never allocates.</p>
 */
public final class Range {
    private final Measure<?> minimum;
    private final Measure<?> maximum;
    private final Unit unit;
    private final double minimumInUnit;
    private final double maximumInUnit;

    private Range(Measure<?> minimum, Measure<?> maximum) {
        this.minimum = Objects.requireNonNull(minimum, "minimum");
        this.maximum = Objects.requireNonNull(maximum, "maximum");
        if (!UnitConversions.sameDimension(minimum.unit(), maximum.unit())) {
            throw new IllegalArgumentException("Range bounds must measure the same dimension.");
        }
        if (!Double.isFinite(minimum.baseUnitMagnitude()) || !Double.isFinite(maximum.baseUnitMagnitude())) {
            throw new IllegalArgumentException("Range bounds must be finite.");
        }
        this.unit = minimum.unit();
        this.minimumInUnit = valueIn(minimum, unit);
        this.maximumInUnit = valueIn(maximum, unit);
        if (maximumInUnit < minimumInUnit) {
            throw new IllegalArgumentException("Range maximum must be greater than or equal to minimum.");
        }
    }

    /**
     * Creates an angular travel range. The bounds keep the unit of {@code minimum}.
     *
     * @param minimum lower bound
     * @param maximum upper bound
     * @return range ref
     */
    public static Range of(Angle minimum, Angle maximum) {
        return new Range(minimum, maximum);
    }

    /**
     * Creates a linear travel range. The bounds keep the unit of {@code minimum}.
     *
     * @param minimum lower bound
     * @param maximum upper bound
     * @return range ref
     */
    public static Range of(Distance minimum, Distance maximum) {
        return new Range(minimum, maximum);
    }

    /**
     * Creates a range from any two bounds of one dimension. The bounds keep the unit of {@code minimum}.
     *
     * @param minimum lower bound
     * @param maximum upper bound
     * @return range ref
     */
    public static Range of(Measure<?> minimum, Measure<?> maximum) {
        return new Range(minimum, maximum);
    }

    /**
     * Creates a range whose bounds are plain numbers in {@code unit}. Used by control bindings, whose
     * declared unit is the unit their positions are expressed in. Use {@link Units#Value} for
     * dimensionless control spaces.
     *
     * @param unit unit the bounds are expressed in
     * @param minimum lower bound in {@code unit}
     * @param maximum upper bound in {@code unit}
     * @return range ref
     */
    public static Range ofUnit(Unit unit, double minimum, double maximum) {
        Objects.requireNonNull(unit, "unit");
        return new Range(unit.of(minimum), unit.of(maximum));
    }

    /** Returns the lower bound as declared. */
    public Measure<?> minimum() {
        return minimum;
    }

    /** Returns the upper bound as declared. */
    public Measure<?> maximum() {
        return maximum;
    }

    /** Returns the unit the bounds were declared in; {@link #clamp(double)} works in this unit. */
    public Unit unit() {
        return unit;
    }

    /** Returns the lower bound as a plain value in {@link #unit()}. */
    double minimumInUnit() {
        return minimumInUnit;
    }

    /** Returns the upper bound as a plain value in {@link #unit()}. */
    double maximumInUnit() {
        return maximumInUnit;
    }

    /**
     * Clamps a value into this range.
     *
     * @param value value in {@link #unit()}
     * @return clamped value in {@link #unit()}
     */
    public double clamp(double value) {
        return Math.max(minimumInUnit, Math.min(maximumInUnit, value));
    }

    private static double valueIn(Measure<?> measure, Unit unit) {
        return measure.unit() == unit
                ? measure.magnitude()
                : unit.fromBaseUnits(measure.baseUnitMagnitude());
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Range range
                && unit.equals(range.unit)
                && Double.compare(minimumInUnit, range.minimumInUnit) == 0
                && Double.compare(maximumInUnit, range.maximumInUnit) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(unit, minimumInUnit, maximumInUnit);
    }

    @Override
    public String toString() {
        return "Range[minimum=" + minimum + ", maximum=" + maximum + "]";
    }

    /** Returns an internal, primitive view of this value. Not for robot code. */
    public Raw raw() {
        return new Raw(this);
    }

    /**
     * Internal, not for robot code: primitive bounds in the range's unit, for Athena's hot paths. Robot code
     * uses minimum() and maximum().
     */
    public static final class Raw {
        private final Range owner;

        private Raw(Range owner) {
            this.owner = owner;
        }

        /** Internal: see Range#minimumInUnit. */
        public double minimumInUnit() {
            return owner.minimumInUnit();
        }

        /** Internal: see Range#maximumInUnit. */
        public double maximumInUnit() {
            return owner.maximumInUnit();
        }
    }
}
