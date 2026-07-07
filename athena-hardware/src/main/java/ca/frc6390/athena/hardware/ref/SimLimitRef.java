package ca.frc6390.athena.hardware.ref;

import java.util.Objects;

/**
 * Simulation-only binding that toggles a digital input near a mechanism position.
 *
 * @param sensor digital input to drive in simulation
 * @param position active position in mechanism units
 * @param tolerance allowed distance from the position
 */
public record SimLimitRef(DigitalInputRef sensor, double position, double tolerance) {
    public SimLimitRef {
        Objects.requireNonNull(sensor, "sensor");
        if (!Double.isFinite(position)) {
            throw new IllegalArgumentException("Sim limit position must be finite.");
        }
        if (!Double.isFinite(tolerance) || tolerance < 0.0) {
            throw new IllegalArgumentException("Sim limit tolerance must be finite and non-negative.");
        }
    }
}
