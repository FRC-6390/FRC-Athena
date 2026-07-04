package ca.frc6390.athena.hardware.ref;

/**
 * Reusable mechanism range.
 *
 * @param minimum minimum value
 * @param maximum maximum value
 * @param unit display/semantic unit
 */
public record RangeRef(double minimum, double maximum, String unit) {
    public RangeRef {
        if (maximum < minimum) {
            throw new IllegalArgumentException("Range maximum must be greater than or equal to minimum.");
        }
        unit = unit == null || unit.isBlank() ? "units" : unit;
    }

    /**
     * Creates a range in degrees.
     *
     * @param minimum minimum degrees
     * @param maximum maximum degrees
     * @return range ref
     */
    public static RangeRef degrees(double minimum, double maximum) {
        return new RangeRef(minimum, maximum, "deg");
    }

    /**
     * Creates a range in rotations.
     *
     * @param minimum minimum rotations
     * @param maximum maximum rotations
     * @return range ref
     */
    public static RangeRef rotations(double minimum, double maximum) {
        return new RangeRef(minimum, maximum, "rot");
    }

    /**
     * Creates a unitless range.
     *
     * @param minimum minimum value
     * @param maximum maximum value
     * @return range ref
     */
    public static RangeRef of(double minimum, double maximum) {
        return new RangeRef(minimum, maximum, "units");
    }

    /**
     * Clamps a value into this range.
     *
     * @param value value
     * @return clamped value
     */
    public double clamp(double value) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
