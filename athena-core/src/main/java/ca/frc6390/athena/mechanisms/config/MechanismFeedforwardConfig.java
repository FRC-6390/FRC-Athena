package ca.frc6390.athena.mechanisms.config;

/**
 * Feedforward profile (named).
 *
 * <p>{@code type} should be one of:
 * simple_motor, arm, elevator.
 *
 * <p>Fields not used by a given type are ignored (treated as 0).
 */
public record MechanismFeedforwardConfig(
        String name,
        String type,
        Double kS,
        Double kG,
        Double kV,
        Double kA,
        Double tolerance
) {
}
