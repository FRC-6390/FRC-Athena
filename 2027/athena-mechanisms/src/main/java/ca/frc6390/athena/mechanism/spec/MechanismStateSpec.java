package ca.frc6390.athena.mechanism.spec;

import java.util.OptionalDouble;

/**
 * Immutable named mechanism state declaration.
 *
 * @param name state name
 * @param target optional numeric setpoint for the mechanism control mode
 */
public record MechanismStateSpec(String name, double target) {
    /** Sentinel for states without numeric targets. */
    public static final double NO_TARGET = Double.NaN;

    public MechanismStateSpec {
        name = name == null || name.isBlank() ? "state" : name;
    }

    /**
     * Returns the optional numeric target.
     *
     * @return optional target
     */
    public OptionalDouble targetValue() {
        return Double.isNaN(target) ? OptionalDouble.empty() : OptionalDouble.of(target);
    }
}
