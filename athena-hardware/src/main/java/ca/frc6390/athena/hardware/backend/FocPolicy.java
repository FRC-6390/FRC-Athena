package ca.frc6390.athena.hardware.backend;

/**
 * Desired field-oriented-control behavior for controllers that support it.
 */
public enum FocPolicy {
    /**
     * Do not request FOC.
     */
    DISABLED,

    /**
     * Request FOC when the device can use it, but permit fallback.
     */
    ENABLE_IF_AVAILABLE,

    /**
     * Require FOC and fail if the device cannot use it.
     */
    REQUIRE
}
