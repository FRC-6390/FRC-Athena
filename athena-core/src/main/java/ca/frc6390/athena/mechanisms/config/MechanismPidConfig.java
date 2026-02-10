package ca.frc6390.athena.mechanisms.config;

/**
 * PID profile (named).
 */
public record MechanismPidConfig(
        String name,
        Double kP,
        Double kI,
        Double kD,
        Double iZone,
        Double period,
        Double tolerance
) {
}

