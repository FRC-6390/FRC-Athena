package ca.frc6390.athena.hardware.input;

/**
 * Origin of an Athena input declaration.
 */
public enum InputSourceKind {
    /** Input comes from a roboRIO digital input channel. */
    DIGITAL_CHANNEL,

    /** Input comes from a roboRIO analog input channel. */
    ANALOG_CHANNEL,

    /** Input is supplied by runtime code. */
    RUNTIME_SUPPLIER,

    /** Input is a constant value. */
    CONSTANT
}
