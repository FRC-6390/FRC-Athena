package ca.frc6390.athena.mechanism.core;

/**
 * Runtime path executor used by the mechanism Action scheduler.
 */
public interface PathRuntime {
    default void initialize(PathAction path, MechanismContext context) {
    }

    default void execute(PathAction path, MechanismContext context) {
    }

    boolean isFinished(PathAction path, MechanismContext context);

    default void end(PathAction path, MechanismContext context, boolean interrupted) {
    }

    static PathRuntime timed(double seconds) {
        return (path, context) -> context.timeInStateSeconds() >= seconds;
    }

    static PathRuntime timedFromPath() {
        return (path, context) -> context.timeInStateSeconds()
                >= path.expectedDurationSeconds().orElse(0.0);
    }
}
