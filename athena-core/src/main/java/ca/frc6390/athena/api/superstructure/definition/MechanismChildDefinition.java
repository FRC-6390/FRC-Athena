package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.Function;

import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;

public record MechanismChildDefinition<SP, E>(
    MechanismDefinition mechanismDefinition,
    Function<SP, E> mapper
) implements SuperstructureChildDefinition<SP> {
    public MechanismChildDefinition {
        mechanismDefinition = Objects.requireNonNull(mechanismDefinition, "mechanismDefinition");
        mapper = Objects.requireNonNull(mapper, "mapper");
    }
}
