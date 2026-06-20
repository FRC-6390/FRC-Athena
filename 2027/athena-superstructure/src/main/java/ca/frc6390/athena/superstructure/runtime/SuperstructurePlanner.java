package ca.frc6390.athena.superstructure.runtime;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.mechanism.spec.MechanismSpec;
import ca.frc6390.athena.mechanism.spec.MechanismStateSpec;
import ca.frc6390.athena.superstructure.spec.SuperstructurePartSpec;
import ca.frc6390.athena.superstructure.spec.SuperstructureSpec;
import ca.frc6390.athena.superstructure.spec.SuperstructureStateSpec;

/**
 * Resolves superstructure states into ordered leaf mechanism targets.
 */
public final class SuperstructurePlanner {
    private final SuperstructureSpec spec;
    private final SuperstructureTransitionGuard guard;

    /**
     * Creates a planner with no transition restrictions.
     *
     * @param spec superstructure spec to plan
     */
    public SuperstructurePlanner(SuperstructureSpec spec) {
        this(spec, SuperstructureTransitionGuard.allowAll());
    }

    /**
     * Creates a planner with an explicit transition guard.
     *
     * @param spec superstructure spec to plan
     * @param guard transition guard
     */
    public SuperstructurePlanner(SuperstructureSpec spec, SuperstructureTransitionGuard guard) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.guard = Objects.requireNonNull(guard, "guard");
    }

    /**
     * Plans a transition when the current state is unknown.
     *
     * @param targetState requested state
     * @return resolved transition plan
     */
    public SuperstructureTransitionPlan plan(String targetState) {
        return plan(null, targetState);
    }

    /**
     * Plans a transition from one superstructure state to another.
     *
     * @param currentState current state, or {@code null} when unknown
     * @param targetState requested state
     * @return resolved transition plan
     */
    public SuperstructureTransitionPlan plan(String currentState, String targetState) {
        Objects.requireNonNull(targetState, "targetState");
        if (currentState != null) {
            requireState(spec, currentState);
        }
        if (!guard.permits(currentState, targetState)) {
            throw new IllegalStateException("Transition from " + currentState + " to " + targetState + " is not permitted.");
        }
        List<SuperstructureMechanismTarget> targets = new ArrayList<>();
        collectTargets(spec, requireState(spec, targetState), "", targets);
        return new SuperstructureTransitionPlan(spec.name(), currentState, targetState, targets);
    }

    private void collectTargets(
            SuperstructureSpec superstructure,
            SuperstructureStateSpec state,
            String pathPrefix,
            List<SuperstructureMechanismTarget> targets) {
        for (SuperstructurePartSpec part : superstructure.parts()) {
            String childStateName = state.partTargets().get(part.name());
            if (childStateName == null) {
                continue;
            }
            String path = pathPrefix.isBlank() ? part.name() : pathPrefix + "." + part.name();
            if (part.kind() == SuperstructurePartSpec.Kind.MECHANISM) {
                targets.add(mechanismTarget(path, part.mechanism(), childStateName));
            } else {
                SuperstructureSpec child = part.superstructure();
                collectTargets(child, requireState(child, childStateName), path, targets);
            }
        }
    }

    private SuperstructureMechanismTarget mechanismTarget(String path, MechanismSpec mechanism, String stateName) {
        MechanismStateSpec state = mechanism.states().stream()
                .filter(candidate -> candidate.name().equals(stateName))
                .findFirst()
                .orElse(null);
        double target = state == null ? MechanismStateSpec.NO_TARGET : state.target();
        return new SuperstructureMechanismTarget(path, mechanism.name(), stateName, target);
    }

    private SuperstructureStateSpec requireState(SuperstructureSpec superstructure, String stateName) {
        return superstructure.states().stream()
                .filter(state -> state.name().equals(stateName))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException(
                        "Unknown state " + stateName + " in superstructure " + superstructure.name() + "."));
    }
}
