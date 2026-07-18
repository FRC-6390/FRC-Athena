package ca.frc6390.athena.mechanism.sysid;

import java.util.Objects;

/** One synchronized characterization sample in rotations or meters. */
public record SysIdSample(
        SysIdState state,
        boolean angular,
        double appliedVoltage,
        double position,
        double velocity,
        double currentAmps) {

    public SysIdSample {
        Objects.requireNonNull(state, "state");
        if (state == SysIdState.NONE) {
            throw new IllegalArgumentException("SysId samples require an active test state.");
        }
        requireFinite(appliedVoltage, "applied voltage");
        requireFinite(position, "position");
        requireFinite(velocity, "velocity");
        if (Double.isInfinite(currentAmps)) {
            throw new IllegalArgumentException("SysId current must be finite or NaN when unavailable.");
        }
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("SysId " + name + " must be finite.");
        }
    }
}
