package ca.frc6390.athena.mechanism.core;

import java.util.Map;
import java.util.List;

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

    /** Returns the stable devices, controls, mechanisms, or actions this path may output to. */
    default List<?> ownership(PathAction path) { return List.of(); }

    /**
     * Returns marker Actions that remain inside this path's scheduler tree, in trigger order. When
     * two active markers drive the same motor or sink, the later entry wins that resource. A marker
     * whose action completes stops being evaluated (see {@link #markerCompleted}); continuous marker
     * actions run until the path ends.
     */
    default Map<String, Action> activeMarkers(PathAction path, MechanismContext context) { return Map.of(); }

    /**
     * Called when a finite marker action completes. Providers should drop the marker from
     * {@link #activeMarkers} so a later occurrence of the same event can trigger it again.
     */
    default void markerCompleted(PathAction path, String marker) {
    }

    /**
     * Returns the names of markers that have not triggered yet, or {@code null} when unknown. Lease
     * reservations only hold resources for pending and running markers.
     */
    default java.util.Collection<String> pendingMarkers(PathAction path) { return null; }

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
