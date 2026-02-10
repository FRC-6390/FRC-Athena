package ca.frc6390.athena.mechanisms.config;

public record MechanismSimElevatorConfig(
        Double drumRadiusM,
        Double carriageMassKg,
        Double minHeightM,
        Double maxHeightM,
        Double startingHeightM,
        Boolean simulateGravity,
        Double nominalVoltage,
        Double unitsPerMeter
) {
}

