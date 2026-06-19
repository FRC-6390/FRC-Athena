package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;
import java.util.Optional;

public record MechanismBooleanInputDefinition(
    String name,
    Optional<Boolean> defaultValue,
    Class<?> declarationType
) implements MechanismInputDefinition {
    public MechanismBooleanInputDefinition {
        name = Objects.requireNonNull(name, "name");
        defaultValue = Objects.requireNonNull(defaultValue, "defaultValue");
        declarationType = Objects.requireNonNull(declarationType, "declarationType");
    }
}
