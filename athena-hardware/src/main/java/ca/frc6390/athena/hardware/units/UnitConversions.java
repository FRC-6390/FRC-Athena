package ca.frc6390.athena.hardware.units;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import java.util.Objects;

/**
 * Declaration-time conversions between WPILib units and the primitive base-unit doubles Athena uses
 * in its control and output paths. Nothing here is meant to be called per cycle with a
 * {@link Measure}; callers convert once and keep the resulting {@code double}.
 */
public final class UnitConversions {
    private UnitConversions() {
    }

    /**
     * Returns whether two units measure the same physical dimension (for example degrees and
     * rotations, or meters and inches). Compared through each unit's base unit.
     *
     * @param first first unit
     * @param second second unit
     * @return true when values can be converted between the units
     */
    public static boolean sameDimension(Unit first, Unit second) {
        Objects.requireNonNull(first, "first");
        Objects.requireNonNull(second, "second");
        return first.getBaseUnit().equals(second.getBaseUnit());
    }

    /**
     * Converts a measure into a plain value in {@code unit}.
     *
     * @param measure measure to convert
     * @param unit target unit; must measure the same dimension as {@code measure}
     * @return value expressed in {@code unit}
     * @throws IllegalArgumentException when the dimensions differ
     */
    public static double value(Measure<?> measure, Unit unit) {
        Objects.requireNonNull(measure, "measure");
        Objects.requireNonNull(unit, "unit");
        if (!sameDimension(measure.unit(), unit)) {
            throw new IllegalArgumentException("Cannot express " + measure.unit().name() + " values in "
                    + unit.name() + ".");
        }
        return unit.fromBaseUnits(measure.baseUnitMagnitude());
    }

    /**
     * Converts a measure of rate (for example angular velocity) into a plain value in
     * {@code unit} per second. The measure must measure the rate of {@code unit}'s dimension.
     *
     * @param rate rate measure
     * @param unit position unit whose per-second rate is wanted
     * @return value expressed in {@code unit} per second
     * @throws IllegalArgumentException when the dimensions differ
     */
    public static double rate(Measure<?> rate, Unit unit) {
        Objects.requireNonNull(rate, "rate");
        Objects.requireNonNull(unit, "unit");
        if (!sameDimension(rate.unit(), perSecond(unit))) {
            throw new IllegalArgumentException("Cannot express " + rate.unit().name() + " values in "
                    + unit.name() + " per second.");
        }
        return rate.baseUnitMagnitude() / baseScale(unit);
    }

    /**
     * Returns the number of base units in one {@code unit}. Used to convert base-unit magnitudes of
     * rates and accelerations, which are not reachable through {@link Unit#fromBaseUnits(double)}.
     *
     * @param unit unit
     * @return base units per unit
     */
    public static double baseScale(Unit unit) {
        return unit.toBaseUnits(1.0);
    }

    /**
     * Returns the multiplier that converts a value in {@code from} into a value in {@code to}.
     *
     * @param from source unit
     * @param to destination unit
     * @return multiplier
     * @throws IllegalArgumentException when the dimensions differ
     */
    public static double factor(Unit from, Unit to) {
        Objects.requireNonNull(from, "from");
        Objects.requireNonNull(to, "to");
        if (!sameDimension(from, to)) {
            throw new IllegalArgumentException("Cannot convert " + from.name() + " to " + to.name() + ".");
        }
        return baseScale(from) / baseScale(to);
    }

    /**
     * Returns the rate unit of {@code unit} per second (for example degrees per second).
     *
     * @param unit position unit
     * @return unit per second
     */
    public static Unit perSecond(Unit unit) {
        return unit.per(Units.Seconds);
    }
}
