package ca.frc6390.athena.api.mechanism.encoder;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import ca.frc6390.athena.api.mechanism.definition.MechanismEncoderDefinition;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.mechanisms.MechanismEncoderUnit;

public final class MechanismEncoder {
    private String name;
    private AthenaEncoder type;
    private Integer id;
    private String bus;
    private Double gearRatio;
    private Double conversion;
    private Double offset;
    private Double conversionOffset;
    private MechanismEncoderUnit unit;
    private Double wrapsEvery;
    private boolean defaultPositionSource;

    private MechanismEncoder() {
    }

    public static MechanismEncoder create() {
        return new MechanismEncoder();
    }

    public static MechanismEncoder create(String name) {
        return create().named(name);
    }

    public static MechanismEncoder from(MechanismEncoderDefinition definition) {
        MechanismEncoder encoder = create()
            .named(definition.name())
            .type(definition.type());
        definition.id().ifPresent(encoder::id);
        definition.bus().ifPresent(encoder::bus);
        if (definition.gearRatio().isPresent()) {
            encoder.gearRatio(definition.gearRatio().getAsDouble());
        }
        if (definition.conversion().isPresent()) {
            encoder.conversion(definition.conversion().getAsDouble());
        }
        if (definition.offset().isPresent()) {
            encoder.offset(definition.offset().getAsDouble());
        }
        if (definition.conversionOffset().isPresent()) {
            encoder.conversionOffset(definition.conversionOffset().getAsDouble());
        }
        definition.unit().ifPresent(encoder::unit);
        if (definition.wrapsEvery().isPresent()) {
            encoder.wrapsEvery(definition.wrapsEvery().getAsDouble());
        }
        if (definition.defaultPositionSource()) {
            encoder.defaultPositionSource();
        }
        return encoder;
    }

    public MechanismEncoder named(String name) {
        this.name = name;
        return this;
    }

    public MechanismEncoder type(AthenaEncoder type) {
        this.type = type;
        return this;
    }

    public MechanismEncoder id(int id) {
        this.id = id;
        return this;
    }

    public MechanismEncoder bus(String bus) {
        this.bus = bus;
        return this;
    }

    public MechanismEncoder gearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    public MechanismEncoder conversion(double conversion) {
        this.conversion = conversion;
        return this;
    }

    public MechanismEncoder offset(double offset) {
        this.offset = offset;
        return this;
    }

    public MechanismEncoder conversionOffset(double conversionOffset) {
        this.conversionOffset = conversionOffset;
        return this;
    }

    public MechanismEncoder unit(MechanismEncoderUnit unit) {
        this.unit = unit;
        return this;
    }

    public MechanismEncoder wrapsEvery(double wrapsEvery) {
        this.wrapsEvery = wrapsEvery;
        return this;
    }

    public MechanismEncoder defaultPositionSource() {
        this.defaultPositionSource = true;
        return this;
    }

    public MechanismEncoderDefinition definition() {
        String resolvedName = name != null && !name.isBlank()
            ? name
            : Integer.toString(Math.abs(requiredId()));
        return new MechanismEncoderDefinition(
            resolvedName,
            requiredType(),
            OptionalInt.of(requiredId()),
            Optional.ofNullable(bus).filter(value -> !value.isBlank()),
            finite(gearRatio),
            finite(conversion),
            finite(offset),
            finite(conversionOffset),
            Optional.ofNullable(unit),
            finite(wrapsEvery),
            defaultPositionSource,
            getClass());
    }

    private static OptionalDouble finite(Double value) {
        return value != null && Double.isFinite(value) ? OptionalDouble.of(value) : OptionalDouble.empty();
    }

    private AthenaEncoder requiredType() {
        if (type == null) {
            throw new IllegalStateException("encoder type is required");
        }
        return type;
    }

    private int requiredId() {
        if (id == null) {
            throw new IllegalStateException("encoder id is required");
        }
        return id;
    }
}
