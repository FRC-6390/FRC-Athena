package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.Function;

import ca.frc6390.athena.mechanisms.SuperstructureMechanism;

public record ExistingNestedSuperstructureChildDefinition<SP, CS, CSP>(
    SuperstructureMechanism<CS, CSP> superstructure,
    Function<SP, CS> mapper
) implements SuperstructureChildDefinition<SP> {
    public ExistingNestedSuperstructureChildDefinition {
        superstructure = Objects.requireNonNull(superstructure, "superstructure");
        mapper = Objects.requireNonNull(mapper, "mapper");
    }
}
