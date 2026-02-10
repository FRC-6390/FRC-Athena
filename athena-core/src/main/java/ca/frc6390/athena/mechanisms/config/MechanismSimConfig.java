package ca.frc6390.athena.mechanisms.config;

/**
 * Simulation configuration.
 *
 * <p>Each sub-table is optional and only one is typically used per mechanism.
 */
public record MechanismSimConfig(
        MechanismSimSimpleMotorConfig simpleMotor,
        MechanismSimArmConfig arm,
        MechanismSimElevatorConfig elevator
) {
}

