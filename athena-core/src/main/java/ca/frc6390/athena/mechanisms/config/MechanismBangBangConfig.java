package ca.frc6390.athena.mechanisms.config;

/**
 * Bang-bang profile (named).
 */
public record MechanismBangBangConfig(
        String name,
        String output,
        Double highOutput,
        Double lowOutput,
        Double tolerance
) {
}
