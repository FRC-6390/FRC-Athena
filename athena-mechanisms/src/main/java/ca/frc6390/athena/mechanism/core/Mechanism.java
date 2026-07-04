package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.mechanism.spec.MechanismSpec;

/**
 * Robot-facing mechanism contract.
 */
public interface Mechanism {
    /**
     * Returns the state this mechanism should start in.
     *
     * @return initial mechanism state
     */
    default MechanismState initialState() {
        MechanismState initialState = MechanismIntrospector.inspect(this).initialState();
        if (initialState == null) {
            throw new IllegalStateException(
                    "Mechanism " + getClass().getName() + " does not declare any MechanismState fields.");
        }
        return initialState;
    }

    /**
     * Allows a mechanism to evolve its active state from runtime lifecycle facts.
     *
     * @param state current state
     * @param ctx mechanism runtime context
     * @return next state
     */
    default MechanismState update(MechanismState state, MechanismContext ctx) {
        return state;
    }

    /**
     * Escape hatch for mechanisms that need imperative behavior.
     *
     * @param state current state
     */
    default void apply(MechanismState state) {
    }

    /**
     * Placeholder for Athena-owned runtime state requests.
     *
     * @param state requested state
     */
    default void set(MechanismState state) {
    }

    /**
     * Optional hardware/control declaration for this mechanism.
     *
     * @return mechanism spec, or null when this mechanism is only compositional
     */
    default MechanismSpec spec() {
        return null;
    }

    /**
     * Lowers this mechanism to a spec.
     *
     * @return mechanism spec, or null
     */
    default MechanismSpec toSpec() {
        return spec();
    }
}
