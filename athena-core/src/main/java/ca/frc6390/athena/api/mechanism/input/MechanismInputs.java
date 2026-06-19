package ca.frc6390.athena.api.mechanism.input;

import java.util.ArrayList;
import java.util.List;

import ca.frc6390.athena.api.mechanism.definition.MechanismBooleanInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDigitalInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDoubleInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismInputDefinition;

public final class MechanismInputs {
    private final List<MechanismInputDefinition> inputs = new ArrayList<>();

    private MechanismInputs() {
    }

    public static MechanismInputs create() {
        return new MechanismInputs();
    }

    public static MechanismInputs from(List<MechanismInputDefinition> definitions) {
        MechanismInputs inputs = create();
        definitions.forEach(definition -> {
            if (definition instanceof MechanismDigitalInputDefinition digital) {
                inputs.add(MechanismDigitalInput.from(digital));
            } else if (definition instanceof MechanismBooleanInputDefinition boolInput) {
                inputs.add(MechanismBooleanInput.from(boolInput));
            } else if (definition instanceof MechanismDoubleInputDefinition doubleInput) {
                inputs.add(MechanismDoubleInput.from(doubleInput));
            }
        });
        return inputs;
    }

    public MechanismInputs digitalInput(String name, int port) {
        return add(MechanismDigitalInput.create(name).port(port));
    }

    public MechanismInputs booleanInput(String name, boolean defaultValue) {
        return add(MechanismBooleanInput.create(name).defaultValue(defaultValue));
    }

    public MechanismInputs doubleInput(String name, double defaultValue) {
        return add(MechanismDoubleInput.create(name).defaultValue(defaultValue));
    }

    public MechanismInputs add(MechanismDigitalInput input) {
        inputs.add(input.definition());
        return this;
    }

    public MechanismInputs add(MechanismBooleanInput input) {
        inputs.add(input.definition());
        return this;
    }

    public MechanismInputs add(MechanismDoubleInput input) {
        inputs.add(input.definition());
        return this;
    }

    public MechanismInputs merge(MechanismInputs other) {
        if (other != null) {
            inputs.addAll(other.inputs);
        }
        return this;
    }

    public List<MechanismInputDefinition> definitions() {
        return List.copyOf(inputs);
    }
}
