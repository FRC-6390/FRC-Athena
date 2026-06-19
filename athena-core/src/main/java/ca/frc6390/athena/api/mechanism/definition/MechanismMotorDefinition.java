package ca.frc6390.athena.api.mechanism.definition;

import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;

public record MechanismMotorDefinition(
    String name,
    AthenaMotor type,
    OptionalInt id,
    Optional<String> bus,
    OptionalDouble currentLimit,
    Optional<MotorNeutralMode> neutralMode,
    Class<?> declarationType
) {
    public MechanismMotorDefinition {
        name = Objects.requireNonNull(name, "name");
        type = Objects.requireNonNull(type, "type");
        bus = Objects.requireNonNull(bus, "bus");
        currentLimit = Objects.requireNonNull(currentLimit, "currentLimit");
        neutralMode = Objects.requireNonNull(neutralMode, "neutralMode");
        declarationType = Objects.requireNonNull(declarationType, "declarationType");
    }
}
