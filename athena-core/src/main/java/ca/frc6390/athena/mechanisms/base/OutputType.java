package ca.frc6390.athena.mechanisms;

/**
 * Defines the output space the mechanism expects.
 */
public enum OutputType {
    /**
     * Output is interpreted as percent (-1..1).
     */
    PERCENT,
    /**
     * Output is interpreted as voltage (volts).
     */
    VOLTAGE,
    /**
     * Output is interpreted as rotations (position setpoint).
     */
    POSITION,
    /**
     * Output is interpreted as rotations per second (velocity setpoint).
     */
    VELOCITY
}
