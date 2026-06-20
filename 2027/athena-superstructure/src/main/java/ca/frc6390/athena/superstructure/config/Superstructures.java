package ca.frc6390.athena.superstructure.config;

/**
 * Entry points for superstructure declarations.
 */
public final class Superstructures {
    private Superstructures() {
    }

    /**
     * Creates a superstructure declaration.
     *
     * @param name superstructure name
     * @return superstructure config
     */
    public static SuperstructureConfig create(String name) {
        return new SuperstructureConfig(name);
    }
}
