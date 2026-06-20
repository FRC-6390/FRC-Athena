package ca.frc6390.athena.mechanism.config;

import ca.frc6390.athena.mechanism.spec.MechanismStateSpec;

/**
 * Student-facing mechanism state builder.
 */
public final class MechanismStateConfig {
    private final String name;
    private double target = MechanismStateSpec.NO_TARGET;

    MechanismStateConfig(String name) {
        this.name = name;
    }

    /**
     * Sets a numeric target for this state.
     *
     * @param target target value
     * @return this config
     */
    public MechanismStateConfig target(double target) {
        this.target = target;
        return this;
    }

    /**
     * Lowers this declaration to an immutable state spec.
     *
     * @return state spec
     */
    public MechanismStateSpec toSpec() {
        return new MechanismStateSpec(name, target);
    }
}
