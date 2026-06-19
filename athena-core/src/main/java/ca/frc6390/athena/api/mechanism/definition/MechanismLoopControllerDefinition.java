package ca.frc6390.athena.api.mechanism.definition;

public sealed interface MechanismLoopControllerDefinition
    permits MechanismBangBangControllerDefinition, MechanismCustomControllerDefinition,
        MechanismFeedforwardControllerDefinition, MechanismPidControllerDefinition {
}
