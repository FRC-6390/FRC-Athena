package ca.frc6390.athena.mechanism.core;

import java.util.Map;

/**
 * Runtime path executor used by the mechanism Action scheduler.
 */
public interface PathRuntime {
    default void initialize(PathAction path, MechanismContext context) {
    }

    default void execute(PathAction path, MechanismContext context) {
    }

    /** Returns the current drivetrain/output Action produced by this path. */
    default Action output(PathAction path, MechanismContext context) { return null; }

    /** Returns marker Actions that remain inside this path's scheduler tree. */
    default Map<String, Action> activeMarkers(PathAction path, MechanismContext context) { return Map.of(); }

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
