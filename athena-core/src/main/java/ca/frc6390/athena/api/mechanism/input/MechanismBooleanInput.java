package ca.frc6390.athena.api.mechanism.input;

import java.util.Optional;

import ca.frc6390.athena.api.mechanism.definition.MechanismBooleanInputDefinition;

public final class MechanismBooleanInput {
    private String name;
    private Boolean defaultValue;

    private MechanismBooleanInput() {
    }

    public static MechanismBooleanInput create() {
        return new MechanismBooleanInput();
    }

    public static MechanismBooleanInput create(String name) {
        return create().named(name);
    }

    public static MechanismBooleanInput from(MechanismBooleanInputDefinition definition) {
        MechanismBooleanInput input = create().named(definition.name());
        definition.defaultValue().ifPresent(input::defaultValue);
        return input;
    }

    public MechanismBooleanInput named(String name) {
        this.name = name;
        return this;
    }

    public MechanismBooleanInput defaultValue(boolean defaultValue) {
        this.defaultValue = defaultValue;
        return this;
    }

    public MechanismBooleanInputDefinition definition() {
        return new MechanismBooleanInputDefinition(requiredName(), Optional.ofNullable(defaultValue), getClass());
    }

    private String requiredName() {
        if (name == null || name.isBlank()) {
            throw new IllegalStateException("boolean input name is required");
        }
        return name;
    }
}
