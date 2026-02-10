package ca.frc6390.athena.mechanisms.config;

public record MechanismSimArmConfig(
        Double armLengthM,
        Double motorReduction,
        Double minDeg,
        Double maxDeg,
        Double startingDeg,
        Double unitsPerRadian,
        Boolean simulateGravity,
        Double nominalVoltage,
        Double momentOfInertia
) {
}

