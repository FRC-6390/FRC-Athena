package ca.frc6390.athena.mechanism.config;

/**
 * Entry points for mechanism declarations.
 */
public final class Mechanisms {
    private Mechanisms() {
    }

    /**
     * Creates a generic simple mechanism.
     *
     * @param name mechanism name
     * @return mechanism config
     */
    public static MechanismConfig simple(String name) {
        return new MechanismConfig(name);
    }

    /**
     * Creates a flywheel mechanism declaration.
     *
     * @param name mechanism name
     * @return mechanism config
     */
    public static MechanismConfig flywheel(String name) {
        return new MechanismConfig(name);
    }
}
