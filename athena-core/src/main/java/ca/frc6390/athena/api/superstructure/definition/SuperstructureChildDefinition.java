package ca.frc6390.athena.api.superstructure.definition;

public sealed interface SuperstructureChildDefinition<SP>
    permits ExistingMechanismChildDefinition, ExistingNestedSuperstructureChildDefinition,
        MechanismChildDefinition, NestedSuperstructureChildDefinition {
}
