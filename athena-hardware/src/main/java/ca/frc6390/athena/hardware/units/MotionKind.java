package ca.frc6390.athena.hardware.units;

/**
 * Kind of motion a unit describes. Derived from the WPILib unit a value is expressed in: angles are
 * {@link #ROTARY}, distances are {@link #LINEAR}, anything else (or no unit) is {@link #UNSPECIFIED}.
 */
public enum MotionKind {
    /** Angular motion (WPILib {@code AngleUnit}). */
    ROTARY,
    /** Linear motion (WPILib {@code DistanceUnit}). */
    LINEAR,
    /** No unit was declared, so the motion kind is unknown. */
    UNSPECIFIED;

    /**
     * Returns the motion kind described by a WPILib unit.
     *
     * @param unit unit, may be null
     * @return rotary for angle units, linear for distance units, otherwise unspecified
     */
    public static MotionKind of(edu.wpi.first.units.Unit unit) {
        if (unit instanceof edu.wpi.first.units.AngleUnit) {
            return ROTARY;
        }
        if (unit instanceof edu.wpi.first.units.DistanceUnit) {
            return LINEAR;
        }
        return UNSPECIFIED;
    }
}
