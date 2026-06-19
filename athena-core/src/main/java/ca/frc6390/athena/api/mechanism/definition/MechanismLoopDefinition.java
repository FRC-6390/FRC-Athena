package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;

import ca.frc6390.athena.mechanisms.OutputType;

public record MechanismLoopDefinition(
    String name,
    OutputType output,
    LoopActivation activation,
    LoopDeclarationKind declarationKind,
    MechanismLoopControllerDefinition controller,
    Class<?> declarationType
) {
    public MechanismLoopDefinition {
        name = Objects.requireNonNull(name, "name");
        output = Objects.requireNonNull(output, "output");
        activation = Objects.requireNonNull(activation, "activation");
        declarationKind = Objects.requireNonNull(declarationKind, "declarationKind");
        controller = Objects.requireNonNull(controller, "controller");
        declarationType = Objects.requireNonNull(declarationType, "declarationType");
    }
}
