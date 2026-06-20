package ca.frc6390.athena.superstructure.spec;

import java.util.Map;
import java.util.Objects;

/**
 * Named superstructure state and per-part targets.
 *
 * @param name state name
 * @param partTargets target labels by part name
 */
public record SuperstructureStateSpec(String name, Map<String, String> partTargets) {
    public SuperstructureStateSpec {
        name = name == null || name.isBlank() ? "state" : name;
        partTargets = Map.copyOf(partTargets);
        Objects.requireNonNull(name, "name");
    }
}
