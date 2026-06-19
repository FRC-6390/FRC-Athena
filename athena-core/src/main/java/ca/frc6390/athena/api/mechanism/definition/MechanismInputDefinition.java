package ca.frc6390.athena.api.mechanism.definition;

public sealed interface MechanismInputDefinition
    permits MechanismBooleanInputDefinition, MechanismDigitalInputDefinition, MechanismDoubleInputDefinition {
    String name();

    Class<?> declarationType();
}
