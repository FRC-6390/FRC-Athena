package ca.frc6390.athena.superstructure.runtime;

import java.util.List;

/**
 * Resolved mechanism targets for a requested superstructure transition.
 *
 * @param superstructureName planned superstructure name
 * @param currentState current superstructure state, or {@code null} when unknown
 * @param targetState requested superstructure state
 * @param targets leaf mechanism targets in declaration order
 */
public record SuperstructureTransitionPlan(
        String superstructureName,
        String currentState,
        String targetState,
        List<SuperstructureMechanismTarget> targets) {
    public SuperstructureTransitionPlan {
        superstructureName = superstructureName == null || superstructureName.isBlank()
                ? "superstructure"
                : superstructureName;
        targetState = targetState == null || targetState.isBlank() ? "state" : targetState;
        targets = List.copyOf(targets);
    }
}
