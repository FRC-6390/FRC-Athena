package ca.frc6390.athena.api.superstructure.definition;

public sealed interface SuperstructureInputDefinition
        permits SuperstructureBooleanInputDefinition,
                SuperstructureDoubleInputDefinition,
                SuperstructureIntInputDefinition,
                SuperstructureStringInputDefinition,
                SuperstructurePose2dInputDefinition,
                SuperstructurePose3dInputDefinition,
                SuperstructureObjectInputDefinition {
    String name();
}
