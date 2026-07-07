package ca.frc6390.athena.mechanism.core;

/**
 * Runtime path executor used by the mechanism state scheduler.
 */
public interface PathRuntime {
    default void initialize(PathRef path, MechanismContext context) {
    }

    default void execute(PathRef path, MechanismContext context) {
    }

    boolean isFinished(PathRef path, MechanismContext context);

    default void end(PathRef path, MechanismContext context, boolean interrupted) {
    }

    static PathRuntime timed(double seconds) {
        return (path, context) -> context.timeInStateSeconds() >= seconds;
    }

    static PathRuntime timedFromRef() {
        return (path, context) -> context.timeInStateSeconds()
                >= path.expectedDurationSeconds().orElse(0.0);
    }
}
