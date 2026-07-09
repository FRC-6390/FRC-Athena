package ca.frc6390.athena.mechanism.core;

/**
 * Runtime path executor used by the mechanism state scheduler.
 */
public interface PathRuntime {
    default void initialize(PathState path, MechanismContext context) {
    }

    default void execute(PathState path, MechanismContext context) {
    }

    boolean isFinished(PathState path, MechanismContext context);

    default void end(PathState path, MechanismContext context, boolean interrupted) {
    }

    static PathRuntime timed(double seconds) {
        return (path, context) -> context.timeInStateSeconds() >= seconds;
    }

    static PathRuntime timedFromPath() {
        return (path, context) -> context.timeInStateSeconds()
                >= path.expectedDurationSeconds().orElse(0.0);
    }
}
