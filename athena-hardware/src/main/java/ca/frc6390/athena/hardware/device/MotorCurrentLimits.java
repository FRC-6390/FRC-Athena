package ca.frc6390.athena.hardware.device;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;

/**
 * Effective current limits configured for a motor.
 *
 * @param supply supply-side limit, zero when disabled
 * @param stator stator-side limit, zero when disabled
 */
public record MotorCurrentLimits(Current supply, Current stator) {
    public MotorCurrentLimits {
        if (supply == null || stator == null) {
            throw new IllegalArgumentException("Motor current limits must be set.");
        }
        if (supply.in(Units.Amps) < 0 || stator.in(Units.Amps) < 0) {
            throw new IllegalArgumentException("Motor current limits cannot be negative.");
        }
    }

    /** Returns the best current limit for detecting a stalled motor. */
    public Current stallDetection() {
        return stator.in(Units.Amps) > 0 ? stator : supply;
    }

    /** Returns whether stall detection can derive a threshold from these limits. */
    public boolean supportsStallDetection() {
        return stallDetection().in(Units.Amps) > 0;
    }
}
