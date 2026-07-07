package ca.frc6390.athena.mechanism.core;

/**
 * How an event turns source state into hook activity.
 */
public enum EventActivation {
    /**
     * Active while the source is true.
     */
    LEVEL,

    /**
     * Active for one tick when the source changes from false to true.
     */
    RISING,

    /**
     * Active for one tick when the source changes from true to false.
     */
    FALLING,

    /**
     * Active for the current runtime tick only.
     */
    PULSE
}
