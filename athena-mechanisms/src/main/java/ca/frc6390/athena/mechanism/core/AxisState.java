package ca.frc6390.athena.mechanism.core;

/**
 * Runtime state for an axis.
 */
public interface AxisState {
    default double position() {
        return 0.0;
    }

    default double velocity() {
        return 0.0;
    }
}
