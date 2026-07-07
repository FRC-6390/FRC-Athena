package ca.frc6390.athena.mechanism.core;

/**
 * Axis behavior when a rule blocks an output request.
 */
public enum BlockPolicy {
    NEUTRAL,
    HOLD_POSITION,
    HOLD_VELOCITY
}
