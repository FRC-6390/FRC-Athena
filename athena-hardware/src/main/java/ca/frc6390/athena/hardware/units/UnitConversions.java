package ca.frc6390.athena.hardware.units;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Unit-dimension checks and factors between WPILib units. Measure values are converted with
 * {@code Measure.in(Unit)}; this class only answers whether two units share a dimension and how
 * base-unit magnitudes relate, for the primitive hot paths that cannot call {@code in}.
 */
public final class UnitConversions {
    // WPILib's derived-unit lookup builds a key even when the unit already exists. These
    // common immutable position units are used for live typed targets. Keep a fixed identity
    // table so arbitrary user-created units do not accumulate in another global cache.
    private static final Map<Unit, Unit> COMMON_RATES = commonRates();

    private UnitConversions() {
    }

    private static Map<Unit, Unit> commonRates() {
        Map<Unit, Unit> rates = new IdentityHashMap<>();
        for (Unit unit : new Unit[] { Units.Rotations, Units.Degrees, Units.Radians,
                Units.Meters, Units.Centimeters, Units.Millimeters, Units.Inches, Units.Feet }) {
            rates.put(unit, unit.per(Units.Seconds));
        }
        return Collections.unmodifiableMap(rates);
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
        Unit common = COMMON_RATES.get(unit);
        return common == null ? unit.per(Units.Seconds) : common;
    }
}
