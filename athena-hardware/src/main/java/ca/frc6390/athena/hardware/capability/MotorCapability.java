package ca.frc6390.athena.hardware.capability;

/**
 * Generic motor features that mechanism validation can require.
 */
public enum MotorCapability {
    /** Open-loop percent output control. */
    PERCENT_OUTPUT,

    /** Direct voltage output control. */
    VOLTAGE_OUTPUT,

    /** Device or adapter can close a position loop. */
    POSITION_CLOSED_LOOP,

    /** Device or adapter can close a velocity loop. */
    VELOCITY_CLOSED_LOOP,

    /** Current limiting can be configured. */
    CURRENT_LIMIT,

    /** Brake/coast neutral mode can be configured. */
    NEUTRAL_MODE,

    /** Device exposes an integrated encoder. */
    INTEGRATED_ENCODER,

    /** Device exposes a controller-attached absolute encoder. */
    ABSOLUTE_ENCODER
}
