package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.Function;

public record NestedSuperstructureChildDefinition<SP, CS, CSP>(
    SuperstructureDefinition<CSP> superstructureDefinition,
    Function<SP, CS> mapper
) implements SuperstructureChildDefinition<SP> {
    public NestedSuperstructureChildDefinition {
        superstructureDefinition = Objects.requireNonNull(superstructureDefinition, "superstructureDefinition");
        mapper = Objects.requireNonNull(mapper, "mapper");
    }
}
