package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.Function;

import ca.frc6390.athena.mechanisms.StatefulMechanism;

public record ExistingMechanismChildDefinition<SP, E>(
    StatefulMechanism<E> mechanism,
    Function<SP, E> mapper
) implements SuperstructureChildDefinition<SP> {
    public ExistingMechanismChildDefinition {
        mechanism = Objects.requireNonNull(mechanism, "mechanism");
        mapper = Objects.requireNonNull(mapper, "mapper");
    }
}
