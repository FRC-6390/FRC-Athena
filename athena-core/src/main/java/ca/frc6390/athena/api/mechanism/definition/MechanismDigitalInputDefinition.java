package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;
import java.util.OptionalDouble;

public record MechanismDigitalInputDefinition(
    String name,
    int port,
    boolean inverted,
    OptionalDouble position,
    boolean hardstop,
    int blockDirection,
    double delaySeconds,
    Class<?> declarationType
) implements MechanismInputDefinition {
    public MechanismDigitalInputDefinition {
        name = Objects.requireNonNull(name, "name");
        position = Objects.requireNonNull(position, "position");
        declarationType = Objects.requireNonNull(declarationType, "declarationType");
    }
}
