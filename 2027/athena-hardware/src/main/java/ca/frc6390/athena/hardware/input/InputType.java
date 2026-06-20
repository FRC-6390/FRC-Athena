package ca.frc6390.athena.hardware.input;

/**
 * Generic input value type.
 */
public enum InputType {
    /** Boolean input, such as a limit switch or runtime predicate. */
    BOOLEAN,

    /** Numeric input, such as an analog sensor or runtime measurement. */
    NUMBER,

    /** Text input, usually used for mode labels or diagnostics. */
    STRING
}
