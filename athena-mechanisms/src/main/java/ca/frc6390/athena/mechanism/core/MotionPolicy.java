package ca.frc6390.athena.mechanism.core;

/**
 * Per-Action motion policy override. A null component inherits the control binding default.
 *
 * @param retarget retarget policy for this Action's target, or null
 * @param release release policy used if this Action is the last one driving its binding, or null
 */
public record MotionPolicy(Retarget retarget, Release release) {
    /** Policy that inherits every binding default. */
    public static final MotionPolicy INHERIT = new MotionPolicy(null, null);
}
