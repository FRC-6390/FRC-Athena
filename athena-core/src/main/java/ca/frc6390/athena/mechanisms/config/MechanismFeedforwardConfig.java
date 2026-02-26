package ca.frc6390.athena.mechanisms.config;

/**
 * Feedforward profile (named).
 *
 * <p>{@code type} should be one of:
 * simple, arm, elevator.
 *
 * <p>Fields not used by a given type are ignored (treated as 0).
 *
 * <p>{@code source} supports:
 * position, velocity, setpoint, or input:&lt;key&gt;.
 */
public record MechanismFeedforwardConfig(
        String name,
        String type,
        Double kS,
        Double kG,
        Double kV,
        Double kA,
        Double tolerance,
        String source
) {
    public MechanismFeedforwardConfig(
            String name,
            String type,
            Double kS,
            Double kG,
            Double kV,
            Double kA,
            Double tolerance) {
        this(name, type, kS, kG, kV, kA, tolerance, null);
    }
}
