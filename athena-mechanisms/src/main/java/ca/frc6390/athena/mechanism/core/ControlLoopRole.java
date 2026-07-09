package ca.frc6390.athena.mechanism.core;

/**
 * How a control loop can participate in an offloaded control plan.
 */
public enum ControlLoopRole {
    /**
     * Loop gains can be configured directly on a motor controller.
     */
    DEVICE_CONFIGURABLE,

    /**
     * Loop output can be added as arbitrary feedforward to a device-owned loop.
     */
    ARBITRARY_FEEDFORWARD,

    /**
     * Loop output changes the requested target before it is sent to hardware.
     */
    TARGET_TRANSFORM,

    /**
     * Loop must run in Athena and produce the final actuator output.
     */
    SOFTWARE_OUTPUT
}
