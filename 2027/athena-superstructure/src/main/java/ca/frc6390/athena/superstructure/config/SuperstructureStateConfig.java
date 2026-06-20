package ca.frc6390.athena.superstructure.config;

import java.util.LinkedHashMap;
import java.util.Map;

import ca.frc6390.athena.superstructure.spec.SuperstructureStateSpec;

/**
 * Student-facing superstructure state declaration.
 */
public final class SuperstructureStateConfig {
    private final String name;
    private final Map<String, String> targets = new LinkedHashMap<>();

    SuperstructureStateConfig(String name) {
        this.name = name;
    }

    /**
     * Assigns a target label to a part.
     *
     * @param partName part name
     * @param target target label
     * @return this config
     */
    public SuperstructureStateConfig part(String partName, String target) {
        targets.put(partName, target);
        return this;
    }

    /**
     * Lowers this state to an immutable spec.
     *
     * @return state spec
     */
    public SuperstructureStateSpec toSpec() {
        return new SuperstructureStateSpec(name, targets);
    }
}
