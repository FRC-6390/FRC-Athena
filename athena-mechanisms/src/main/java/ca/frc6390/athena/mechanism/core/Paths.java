package ca.frc6390.athena.mechanism.core;

/**
 * Factories for autonomous path Actions.
 */
public final class Paths {
    private Paths() {
    }

    public static PathAction of(String source, String name) {
        return new PathAction(source, name);
    }

    public static PathAction choreo(String name) {
        return of("choreo", name);
    }

    public static PathAction pathPlanner(String name) {
        return of("pathplanner", name);
    }
}
