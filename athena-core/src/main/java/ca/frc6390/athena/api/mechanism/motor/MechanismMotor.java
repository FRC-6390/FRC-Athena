package ca.frc6390.athena.api.mechanism.motor;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import ca.frc6390.athena.api.mechanism.definition.MechanismMotorDefinition;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;

public final class MechanismMotor {
    private String name;
    private AthenaMotor type;
    private Integer id;
    private String bus;
    private Double currentLimit;
    private MotorNeutralMode neutralMode;

    private MechanismMotor() {
    }

    public static MechanismMotor create() {
        return new MechanismMotor();
    }

    public static MechanismMotor create(String name) {
        return create().named(name);
    }

    public static MechanismMotor from(MechanismMotorDefinition definition) {
        return create()
            .named(definition.name())
            .type(definition.type())
            .id(definition.id().orElse(0))
            .bus(definition.bus().orElse(null))
            .currentLimit(definition.currentLimit().isPresent() ? definition.currentLimit().getAsDouble() : Double.NaN)
            .neutralMode(definition.neutralMode().orElse(null));
    }

    public MechanismMotor named(String name) {
        this.name = name;
        return this;
    }

    public MechanismMotor type(AthenaMotor type) {
        this.type = type;
        return this;
    }

    public MechanismMotor id(int id) {
        this.id = id;
        return this;
    }

    public MechanismMotor bus(String bus) {
        this.bus = bus;
        return this;
    }

    public MechanismMotor currentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public MechanismMotor neutralMode(MotorNeutralMode neutralMode) {
        this.neutralMode = neutralMode;
        return this;
    }

    public MechanismMotorDefinition definition() {
        String resolvedName = name != null && !name.isBlank()
            ? name
            : Integer.toString(Math.abs(requiredId()));
        return new MechanismMotorDefinition(
            resolvedName,
            requiredType(),
            OptionalInt.of(requiredId()),
            Optional.ofNullable(bus).filter(value -> !value.isBlank()),
            currentLimit != null && Double.isFinite(currentLimit) ? OptionalDouble.of(currentLimit) : OptionalDouble.empty(),
            Optional.ofNullable(neutralMode),
            getClass());
    }

    private AthenaMotor requiredType() {
        if (type == null) {
            throw new IllegalStateException("motor type is required");
        }
        return type;
    }

    private int requiredId() {
        if (id == null) {
            throw new IllegalStateException("motor id is required");
        }
        return id;
    }
}
