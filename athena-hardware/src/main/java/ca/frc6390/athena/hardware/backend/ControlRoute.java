package ca.frc6390.athena.hardware.backend;

/**
 * Runtime owner selected for a motor control request.
 */
public enum ControlRoute {
    /**
     * Percent or voltage output with no closed-loop controller.
     */
    OPEN_LOOP,

    /**
     * Athena calculates the closed-loop output before sending it to hardware.
     */
    ATHENA_CLOSED_LOOP,

    /**
     * The motor controller owns the closed-loop calculation.
     */
    DEVICE_CLOSED_LOOP,

    /**
     * The motor controller owns the inner loop while Athena supplies additive corrections.
     */
    HYBRID_CLOSED_LOOP
}
