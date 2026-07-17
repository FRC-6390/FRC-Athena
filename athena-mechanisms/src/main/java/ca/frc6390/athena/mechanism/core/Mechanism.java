package ca.frc6390.athena.mechanism.core;

/**
 * Robot-facing mechanism contract.
 */
public interface Mechanism {
    default Action update(Action action, MechanismContext ctx) {
        return action;
    }

    default void apply(Action action) {
    }

    default void set(Action action) {
    }
}
