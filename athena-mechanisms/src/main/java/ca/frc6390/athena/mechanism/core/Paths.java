package ca.frc6390.athena.mechanism.core;

/**
 * Factories for autonomous path refs.
 */
public final class Paths {
    private Paths() {
    }

    public static PathRef of(String source, String name) {
        return new PathRef(source, name);
    }

    public static PathRef choreo(String name) {
        return of("choreo", name);
    }

    public static PathRef pathplanner(String name) {
        return of("pathplanner", name);
    }
}
