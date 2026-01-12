package ca.frc6390.athena.core;

/**
 * Team-facing auto engine catalog keyed by string identifiers. Resolves to concrete engines via
 * {@link AutoRegistry}.
 */
public enum AthenaAutoEngine {
    PATHPLANNER("auto:pathplanner"),
    CHOREO("auto:choreo");

    private final String key;

    AthenaAutoEngine(String key) {
        this.key = key;
    }

    public RobotAuto.AutoSource resolve() {
        return AutoRegistry.get().engine(key);
    }
}
