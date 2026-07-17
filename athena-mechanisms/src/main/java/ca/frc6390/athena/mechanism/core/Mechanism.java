package ca.frc6390.athena.mechanism.core;

/**
 * Robot-facing mechanism contract.
 */
public interface Mechanism {
    /**
     * Assigns this mechanism as the owner of an action whose ownership cannot be
     * inferred from hardware or control declarations.
     */
    default Action own(Action action) {
        return Actions.owned(this, action);
    }

    default Action update(Action action, MechanismContext ctx) {
        return action;
    }

    default void apply(Action action) {
    }

    default void set(Action action) {
    }
}
