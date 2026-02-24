package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.AutoRegistry;
import ca.frc6390.athena.core.RobotAuto;

/**
 * Example usage of the auto-engine provider registry.
 */
public final class AutoRegistryExamples {
    private AutoRegistryExamples() {}

    public static AutoRegistry registry() {
        return AutoRegistry.get();
    }

    public static AutoRegistry registerCustomEngine(String key) {
        return AutoRegistry.get().add(key, RobotAuto.AutoSource.CUSTOM);
    }

    public static RobotAuto.AutoSource requireEngine(String key) {
        return AutoRegistry.get().engine(key);
    }
}
