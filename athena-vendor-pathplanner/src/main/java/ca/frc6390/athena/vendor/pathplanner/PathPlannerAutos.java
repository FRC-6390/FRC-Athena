package ca.frc6390.athena.vendor.pathplanner;

import ca.frc6390.athena.auto.AutoRegistry;

/**
 * Registration helpers for PathPlanner autonomous sources.
 */
public final class PathPlannerAutos {
    private PathPlannerAutos() {
    }

    /**
     * Registers the PathPlanner source in a registry.
     *
     * @param registry auto registry
     * @return registry
     */
    public static AutoRegistry register(AutoRegistry registry) {
        AutoRegistry target = registry == null ? AutoRegistry.get() : registry;
        target.register(PathPlannerAutoSource.KEY, new PathPlannerAutoSource());
        return target;
    }
}
