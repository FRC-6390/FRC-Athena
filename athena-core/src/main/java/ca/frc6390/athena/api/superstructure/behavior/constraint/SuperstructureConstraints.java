package ca.frc6390.athena.api.superstructure.behavior.constraint;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Predicate;

import ca.frc6390.athena.api.superstructure.definition.SuperstructureConstraintDefinition;
import ca.frc6390.athena.mechanisms.SuperstructureContext;
import ca.frc6390.athena.mechanisms.statespec.StateNames;

public final class SuperstructureConstraints<S, SP> {
    private final List<SuperstructureConstraintDefinition<SP>> definitions = new ArrayList<>();

    private SuperstructureConstraints() {
    }

    public static <S, SP> SuperstructureConstraints<S, SP> create() {
        return new SuperstructureConstraints<>();
    }

    public static <S, SP> SuperstructureConstraints<S, SP> from(
            List<SuperstructureConstraintDefinition<SP>> definitions) {
        SuperstructureConstraints<S, SP> constraints = create();
        if (definitions != null) {
            constraints.definitions.addAll(definitions);
        }
        return constraints;
    }

    @SafeVarargs
    public final SuperstructureConstraints<S, SP> state(
            S state,
            Predicate<SuperstructureContext<SP>> guard,
            S... transitionStates) {
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(guard, "guard");
        List<String> transitions = new ArrayList<>();
        if (transitionStates != null) {
            for (S transition : transitionStates) {
                if (transition != null) {
                    transitions.add(StateNames.name(transition));
                }
            }
        }
        definitions.add(new SuperstructureConstraintDefinition<>(StateNames.name(state), guard, transitions));
        return this;
    }

    public SuperstructureConstraints<S, SP> merge(SuperstructureConstraints<S, SP> other) {
        if (other != null) {
            definitions.addAll(other.definitions);
        }
        return this;
    }

    public List<SuperstructureConstraintDefinition<SP>> definitions() {
        return List.copyOf(definitions);
    }
}
