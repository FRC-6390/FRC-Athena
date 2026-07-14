package ca.frc6390.athena.runtime;

/** Controls fail-fast checks for optional vendor runtime dependencies. */
public final class RuntimeDependencyChecks {
    private static volatile boolean enabled = Boolean.parseBoolean(
            System.getProperty("athena.runtime.dependencyChecks", "true"));

    private RuntimeDependencyChecks() {
    }

    public static boolean enabled() {
        return enabled;
    }

    public static void enabled(boolean enabled) {
        RuntimeDependencyChecks.enabled = enabled;
    }
}
