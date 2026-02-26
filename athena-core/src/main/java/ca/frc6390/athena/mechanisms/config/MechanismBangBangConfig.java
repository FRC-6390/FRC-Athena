package ca.frc6390.athena.mechanisms.config;

/**
 * Bang-bang profile (named).
 *
 * <p>{@code source} supports:
 * position, velocity, setpoint, or input:&lt;key&gt;.
 */
public record MechanismBangBangConfig(
        String name,
        String output,
        Double highOutput,
        Double lowOutput,
        Double tolerance,
        String source
) {
    public MechanismBangBangConfig(
            String name,
            String output,
            Double highOutput,
            Double lowOutput,
            Double tolerance) {
        this(name, output, highOutput, lowOutput, tolerance, null);
    }
}
