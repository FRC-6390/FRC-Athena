package ca.frc6390.athena.hardware.sensor;

/**
 * Generic sensor wrapper kind.
 */
public enum SensorKind {
    /** Limit switch with optional hardstop metadata. */
    LIMIT_SWITCH,

    /** Human-operated button or trigger. */
    BUTTON,

    /** Beam-break sensor. */
    BEAM_BREAK
}
