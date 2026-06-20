package ca.frc6390.athena.mechanism.spec;

/**
 * Mechanism-level control mode.
 */
public enum ControlMode {
    /** No control mode declared. */
    NONE,

    /** Open-loop percent output. */
    PERCENT_OUTPUT,

    /** Closed-loop position control. */
    POSITION,

    /** Closed-loop velocity control. */
    VELOCITY
}
