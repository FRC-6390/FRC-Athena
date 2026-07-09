package ca.frc6390.athena.vision.runtime;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Athena-owned vision simulation field description.
 *
 * @param targets field targets visible to camera simulation providers
 */
public record VisionSimulationField(List<VisionSimulationTarget> targets) {
    public static final VisionSimulationField EMPTY = new VisionSimulationField(List.of());

    public VisionSimulationField {
        targets = targets == null
                ? List.of()
                : targets.stream().filter(Objects::nonNull).toList();
    }

    /**
     * Creates a field with the supplied targets.
     *
     * @param targets targets
     * @return field
     */
    public static VisionSimulationField of(VisionSimulationTarget... targets) {
        return new VisionSimulationField(targets == null ? List.of() : List.of(targets));
    }

    /**
     * Returns a copy with an additional target.
     *
     * @param target target
     * @return updated field
     */
    public VisionSimulationField withTarget(VisionSimulationTarget target) {
        if (target == null) {
            return this;
        }
        ArrayList<VisionSimulationTarget> updated = new ArrayList<>(targets);
        updated.add(target);
        return new VisionSimulationField(updated);
    }
}
