package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.mechanisms.MechanismEncoderUnit;

public record MechanismEncoderDefinition(
    String name,
    AthenaEncoder type,
    OptionalInt id,
    Optional<String> bus,
    OptionalDouble gearRatio,
    OptionalDouble conversion,
    OptionalDouble offset,
    OptionalDouble conversionOffset,
    Optional<MechanismEncoderUnit> unit,
    OptionalDouble wrapsEvery,
    boolean defaultPositionSource,
    Class<?> declarationType
) {
    public MechanismEncoderDefinition {
        name = Objects.requireNonNull(name, "name");
        type = Objects.requireNonNull(type, "type");
        bus = Objects.requireNonNull(bus, "bus");
        gearRatio = Objects.requireNonNull(gearRatio, "gearRatio");
        conversion = Objects.requireNonNull(conversion, "conversion");
        offset = Objects.requireNonNull(offset, "offset");
        conversionOffset = Objects.requireNonNull(conversionOffset, "conversionOffset");
        unit = Objects.requireNonNull(unit, "unit");
        wrapsEvery = Objects.requireNonNull(wrapsEvery, "wrapsEvery");
        declarationType = Objects.requireNonNull(declarationType, "declarationType");
    }
}
