package ca.frc6390.athena.mechanism.core;

/**
 * Robot-facing mechanism contract.
 */
public interface Mechanism {
    default State initialState() {
        State initialState = MechanismIntrospector.inspect(this).initialState();
        if (initialState == null) {
            throw new IllegalStateException("Mechanism " + getClass().getName() + " does not declare any State fields.");
        }
        return initialState;
    }

    default State update(State state, MechanismContext ctx) {
        return state;
    }

    default void apply(State state) {
    }

    default void set(State state) {
    }
}
