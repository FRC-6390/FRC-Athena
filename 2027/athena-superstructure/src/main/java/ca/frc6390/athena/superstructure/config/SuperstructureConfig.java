package ca.frc6390.athena.superstructure.config;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import ca.frc6390.athena.mechanism.config.MechanismConfig;
import ca.frc6390.athena.superstructure.spec.SuperstructurePartSpec;
import ca.frc6390.athena.superstructure.spec.SuperstructureSpec;

/**
 * Student-facing superstructure declaration.
 */
public final class SuperstructureConfig {
    private final String name;
    private final List<PartConfig> parts = new ArrayList<>();
    private final List<SuperstructureStateConfig> states = new ArrayList<>();

    SuperstructureConfig(String name) {
        this.name = name;
    }

    /**
     * Adds a mechanism part.
     *
     * @param name part name
     * @param mechanism mechanism config
     * @return this config
     */
    public SuperstructureConfig part(String name, MechanismConfig mechanism) {
        parts.add(new PartConfig(name, mechanism, null));
        return this;
    }

    /**
     * Adds a nested superstructure part.
     *
     * @param name part name
     * @param superstructure nested superstructure config
     * @return this config
     */
    public SuperstructureConfig part(String name, SuperstructureConfig superstructure) {
        parts.add(new PartConfig(name, null, superstructure));
        return this;
    }

    /**
     * Adds a named superstructure state.
     *
     * @param name state name
     * @param configure state configuration
     * @return this config
     */
    public SuperstructureConfig state(String name, Consumer<SuperstructureStateConfig> configure) {
        SuperstructureStateConfig state = new SuperstructureStateConfig(name);
        if (configure != null) {
            configure.accept(state);
        }
        states.add(state);
        return this;
    }

    /**
     * Lowers this declaration to an immutable spec.
     *
     * @return superstructure spec
     */
    public SuperstructureSpec toSpec() {
        List<SuperstructurePartSpec> partSpecs = parts.stream()
                .map(part -> part.mechanism() != null
                        ? new SuperstructurePartSpec(part.name(), part.mechanism().toSpec())
                        : new SuperstructurePartSpec(part.name(), part.superstructure().toSpec()))
                .toList();
        return new SuperstructureSpec(
                name,
                partSpecs,
                states.stream().map(SuperstructureStateConfig::toSpec).toList());
    }

    private record PartConfig(String name, MechanismConfig mechanism, SuperstructureConfig superstructure) {
    }
}
