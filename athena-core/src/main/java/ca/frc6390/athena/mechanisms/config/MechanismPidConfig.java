package ca.frc6390.athena.mechanisms.config;

/**
 * PID profile (named).
 *
 * <p>{@code source} supports:
 * position, velocity, setpoint, or input:&lt;key&gt;.
 *
 * <p>Profiled PID is enabled when both {@code maxVelocity} and {@code maxAcceleration}
 * are provided.
 */
public record MechanismPidConfig(
        String name,
        Double kP,
        Double kI,
        Double kD,
        Double iZone,
        Double period,
        Double tolerance,
        Double maxVelocity,
        Double maxAcceleration,
        String source
) {
    public MechanismPidConfig(
            String name,
            Double kP,
            Double kI,
            Double kD,
            Double iZone,
            Double period,
            Double tolerance) {
        this(name, kP, kI, kD, iZone, period, tolerance, null, null, null);
    }

    public MechanismPidConfig(
            String name,
            Double kP,
            Double kI,
            Double kD,
            Double iZone,
            Double period,
            Double tolerance,
            String source) {
        this(name, kP, kI, kD, iZone, period, tolerance, null, null, source);
    }
}
