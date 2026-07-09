package ca.frc6390.athena.mechanism.core;

/**
 * Robot-facing mechanism contract.
 */
public interface Mechanism {
    default Action initialState() {
        Action initialState = MechanismIntrospector.inspect(this).initialState();
        if (initialState == null) {
            throw new IllegalStateException("Mechanism " + getClass().getName() + " does not declare any Action fields.");
        }
        return initialState;
    }

    default Action update(Action action, MechanismContext ctx) {
        return action;
    }

    default void apply(Action action) {
    }

    default void set(Action action) {
    }
}
