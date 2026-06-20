package ca.frc6390.athena.vendor.choreo;

import ca.frc6390.athena.auto.AutoRegistry;

/**
 * Registration helpers for Choreo autonomous sources.
 */
public final class ChoreoAutos {
    private ChoreoAutos() {
    }

    /**
     * Registers the Choreo source in a registry.
     *
     * @param registry auto registry
     * @return registry
     */
    public static AutoRegistry register(AutoRegistry registry) {
        AutoRegistry target = registry == null ? AutoRegistry.get() : registry;
        target.register(ChoreoAutoSource.KEY, new ChoreoAutoSource());
        return target;
    }
}
