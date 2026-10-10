package ca.frc6390.athena.mechanism.model;

import ca.frc6390.athena.mechanism.sysid.SysIdState;
import java.util.Objects;

/**
 * One characterization sample in base units: rotations (rotary) or meters (linear), per second, per
 * second squared, and volts.
 *
 * @param state test phase that produced the sample
 * @param timeSeconds monotonic time of the sample
 * @param voltage voltage applied by the test
 * @param position mechanism position
 * @param velocity mechanism velocity
 * @param acceleration acceleration from successive samples of the same test (0 for the first)
 * @param currentAmps stator or supply current, NaN when unavailable
 */
public record CharacterizationSample(
        SysIdState state,
        double timeSeconds,
        double voltage,
        double position,
        double velocity,
        double acceleration,
        double currentAmps) {
    public CharacterizationSample {
        Objects.requireNonNull(state, "state");
        if (state == SysIdState.NONE) {
            throw new IllegalArgumentException("Characterization samples require an active test state.");
        }
        requireFinite(timeSeconds, "time");
        requireFinite(voltage, "voltage");
        requireFinite(position, "position");
        requireFinite(velocity, "velocity");
        requireFinite(acceleration, "acceleration");
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Characterization " + name + " must be finite.");
        }
    }
}
