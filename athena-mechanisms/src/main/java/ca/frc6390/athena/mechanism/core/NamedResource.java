package ca.frc6390.athena.mechanism.core;

import java.util.Objects;

/**
 * Lease key for a resource that Athena does not model as a device, control, or mechanism, such as a
 * WPILib subsystem. Two Actions that require the same name conflict exactly like two Actions that drive
 * the same motor: the newer request wins and an overlapping {@link LifecycleAction} ends. Equality is by
 * name, so the same name always identifies the same resource.
 *
 * @param name stable, non-blank resource name
 */
public record NamedResource(String name) {
    public NamedResource {
        Objects.requireNonNull(name, "name");
        name = name.trim();
        if (name.isEmpty()) throw new IllegalArgumentException("Named resource requires a non-blank name.");
    }

    /**
     * Creates a named resource.
     *
     * @param name stable, non-blank resource name
     * @return named resource
     */
    public static NamedResource of(String name) {
        return new NamedResource(name);
    }
}
