package ca.frc6390.athena.mechanisms.config;

/**
 * Single motor controller entry.
 *
 * <p>The {@code motor} field is expected to match an {@code AthenaMotor} enum name (for example
 * {@code KRAKEN_X60}). This keeps deploy configs vendor-agnostic.
 *
 * <p>The {@code type} field is an alternative that directly references a {@link ca.frc6390.athena.hardware.motor.MotorControllerType}
 * registry key (for example {@code talonfx}). This is used by data-only exports where the original
 * {@code AthenaMotor} model is not available.
 */
public record MechanismMotorConfig(
        String motor,
        String type,
        Integer id,
        Boolean inverted
) {
}
