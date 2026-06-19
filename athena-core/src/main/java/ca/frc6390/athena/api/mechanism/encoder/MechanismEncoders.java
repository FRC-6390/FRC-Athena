package ca.frc6390.athena.api.mechanism.encoder;

import java.util.ArrayList;
import java.util.List;

import ca.frc6390.athena.api.mechanism.definition.MechanismEncoderDefinition;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;

public final class MechanismEncoders {
    private final List<MechanismEncoder> encoders = new ArrayList<>();

    private MechanismEncoders() {
    }

    public static MechanismEncoders create() {
        return new MechanismEncoders();
    }

    public static MechanismEncoders from(List<MechanismEncoderDefinition> definitions) {
        MechanismEncoders encoders = create();
        definitions.forEach(definition -> encoders.add(MechanismEncoder.from(definition)));
        return encoders;
    }

    public MechanismEncoders add(AthenaEncoder type, int id) {
        return add(MechanismEncoder.create().type(type).id(id));
    }

    public MechanismEncoders add(String name, AthenaEncoder type, int id) {
        return add(MechanismEncoder.create(name).type(type).id(id));
    }

    public MechanismEncoders add(MechanismEncoder encoder) {
        encoders.add(encoder);
        return this;
    }

    public MechanismEncoders merge(MechanismEncoders other) {
        if (other != null) {
            other.encoders.forEach(this::add);
        }
        return this;
    }

    public List<MechanismEncoderDefinition> definitions() {
        return encoders.stream().map(MechanismEncoder::definition).toList();
    }
}
