package ca.frc6390.athena.mechanism.model;

import ca.frc6390.athena.hardware.units.MotionKind;

/**
 * Gravity term fitted alongside friction and inertia.
 *
 * <ul>
 *   <li>{@link #NONE}: no gravity. Flywheels, rollers and turrets. The fit has no gravity column, so kG is zero.</li>
 *   <li>{@link #CONSTANT}: a constant voltage, for example an elevator carriage.</li>
 *   <li>{@link #COSINE}: an amplitude times {@code cos(2 pi p + offset)}, for example an arm pivot or a hood.</li>
 * </ul>
 */
public enum Gravity {
    NONE,
    CONSTANT,
    COSINE;

    /** The default for a motion kind when the declaration does not choose one: rotary is cosine, otherwise constant. */
    public static Gravity of(MotionKind kind) {
        return kind == MotionKind.ROTARY ? COSINE : CONSTANT;
    }
}
