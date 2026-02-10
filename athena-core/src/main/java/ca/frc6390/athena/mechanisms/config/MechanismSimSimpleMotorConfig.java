package ca.frc6390.athena.mechanisms.config;

public record MechanismSimSimpleMotorConfig(
        Double momentOfInertia,
        Double nominalVoltage,
        Double unitsPerRadian
) {
}

