package ca.frc6390.athena.api.mechanism.input;

import java.util.OptionalDouble;

import ca.frc6390.athena.api.mechanism.definition.MechanismDoubleInputDefinition;

public final class MechanismDoubleInput {
    private String name;
    private Double defaultValue;

    private MechanismDoubleInput() {
    }

    public static MechanismDoubleInput create() {
        return new MechanismDoubleInput();
    }

    public static MechanismDoubleInput create(String name) {
        return create().named(name);
    }

    public static MechanismDoubleInput from(MechanismDoubleInputDefinition definition) {
        MechanismDoubleInput input = create().named(definition.name());
        if (definition.defaultValue().isPresent()) {
            input.defaultValue(definition.defaultValue().getAsDouble());
        }
        return input;
    }

    public MechanismDoubleInput named(String name) {
        this.name = name;
        return this;
    }

    public MechanismDoubleInput defaultValue(double defaultValue) {
        this.defaultValue = defaultValue;
        return this;
    }

    public MechanismDoubleInputDefinition definition() {
        return new MechanismDoubleInputDefinition(
            requiredName(),
            defaultValue != null ? OptionalDouble.of(defaultValue) : OptionalDouble.empty(),
            getClass());
    }

    private String requiredName() {
        if (name == null || name.isBlank()) {
            throw new IllegalStateException("double input name is required");
        }
        return name;
    }
}
