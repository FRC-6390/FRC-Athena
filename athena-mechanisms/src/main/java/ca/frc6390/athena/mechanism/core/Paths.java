package ca.frc6390.athena.mechanism.core;

/**
 * Factories for autonomous path states.
 */
public final class Paths {
    private Paths() {
    }

    public static PathState of(String source, String name) {
        return new PathState(source, name);
    }

    public static PathState choreo(String name) {
        return of("choreo", name);
    }

    public static PathState pathPlanner(String name) {
        return of("pathplanner", name);
    }
}
