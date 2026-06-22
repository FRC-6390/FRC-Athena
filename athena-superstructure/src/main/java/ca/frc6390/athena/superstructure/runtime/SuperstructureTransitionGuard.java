package ca.frc6390.athena.superstructure.runtime;

/**
 * Runtime policy hook for allowing or rejecting a superstructure transition.
 */
@FunctionalInterface
public interface SuperstructureTransitionGuard {
    /**
     * Returns whether a transition may be planned.
     *
     * @param currentState current superstructure state, or {@code null} when unknown
     * @param targetState requested superstructure state
     * @return true when the transition is permitted
     */
    boolean permits(String currentState, String targetState);

    /**
     * Creates a guard that allows every transition.
     *
     * @return permissive transition guard
     */
    static SuperstructureTransitionGuard allowAll() {
        return (currentState, targetState) -> true;
    }
}
