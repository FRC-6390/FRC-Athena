package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;
import java.util.OptionalDouble;

public record MechanismDoubleInputDefinition(
    String name,
    OptionalDouble defaultValue,
    Class<?> declarationType
) implements MechanismInputDefinition {
    public MechanismDoubleInputDefinition {
        name = Objects.requireNonNull(name, "name");
        defaultValue = Objects.requireNonNull(defaultValue, "defaultValue");
        declarationType = Objects.requireNonNull(declarationType, "declarationType");
    }
}
