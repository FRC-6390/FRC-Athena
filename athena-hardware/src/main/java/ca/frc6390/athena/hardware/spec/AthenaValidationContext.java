package ca.frc6390.athena.hardware.spec;

import ca.frc6390.athena.hardware.backend.BackendRegistry;

/**
 * Context used by spec validation.
 *
 * @param backendRegistry installed backend registry
 */
public record AthenaValidationContext(BackendRegistry backendRegistry) {
    /**
     * Returns the default global context.
     *
     * @return global validation context
     */
    public static AthenaValidationContext global() {
        return new AthenaValidationContext(BackendRegistry.global());
    }

    /**
     * Creates a context with an explicit backend registry.
     *
     * @param registry backend registry
     * @return validation context
     */
    public static AthenaValidationContext withBackends(BackendRegistry registry) {
        return new AthenaValidationContext(registry);
    }

    public AthenaValidationContext {
        backendRegistry = backendRegistry == null ? BackendRegistry.global() : backendRegistry;
    }
}
