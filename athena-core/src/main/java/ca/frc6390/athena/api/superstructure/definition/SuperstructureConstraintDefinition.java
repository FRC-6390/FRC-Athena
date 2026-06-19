package ca.frc6390.athena.api.superstructure.definition;

import java.util.List;
import java.util.Objects;
import java.util.function.Predicate;

import ca.frc6390.athena.mechanisms.SuperstructureContext;

public record SuperstructureConstraintDefinition<SP>(
    String state,
    Predicate<SuperstructureContext<SP>> guard,
    List<String> transitionStates
) {
    public SuperstructureConstraintDefinition {
        state = Objects.requireNonNull(state, "state");
        guard = Objects.requireNonNull(guard, "guard");
        transitionStates = List.copyOf(Objects.requireNonNull(transitionStates, "transitionStates"));
    }
}
