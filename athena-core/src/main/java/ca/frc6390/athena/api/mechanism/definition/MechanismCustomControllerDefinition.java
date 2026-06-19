package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;
import java.util.Optional;

import ca.frc6390.athena.api.mechanism.behavior.control.MechanismLoopCallback;

public record MechanismCustomControllerDefinition(
    Optional<MechanismLoopCallback> callback
) implements MechanismLoopControllerDefinition {
    public static final MechanismCustomControllerDefinition EMPTY =
        new MechanismCustomControllerDefinition(Optional.empty());

    public MechanismCustomControllerDefinition {
        callback = Objects.requireNonNullElse(callback, Optional.empty());
    }

    public static MechanismCustomControllerDefinition of(MechanismLoopCallback callback) {
        return new MechanismCustomControllerDefinition(Optional.of(Objects.requireNonNull(callback, "callback")));
    }
}
