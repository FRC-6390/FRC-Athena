package ca.frc6390.athena.vision.runtime;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Athena-owned vision simulation field description.
 *
 * @param targets field targets visible to camera simulation providers
 * @param noise frame timing, noise and loss model; {@link VisionSimNoise#realistic()} when null
 */
public record VisionSimulationField(List<VisionSimulationTarget> targets, VisionSimNoise noise) {
    public static final VisionSimulationField EMPTY = new VisionSimulationField(List.of());

    /**
     * Creates a field with the realistic noise model.
     *
     * @param targets field targets
     */
    public VisionSimulationField(List<VisionSimulationTarget> targets) {
        this(targets, null);
    }

    public VisionSimulationField {
        targets = targets == null
                ? List.of()
                : targets.stream().filter(Objects::nonNull).toList();
        noise = noise == null ? VisionSimNoise.realistic() : noise;
    }

    /**
     * Creates a field with the supplied targets and the realistic noise model.
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
        return new VisionSimulationField(updated, noise);
    }

    /**
     * Returns a copy that simulates cameras with the supplied noise model.
     *
     * <pre>{@code
     * VisionSimulationField.of(tags).noise(VisionSimNoise.perfect())  // pipeline-only tests
     * }</pre>
     *
     * @param model noise model; {@link VisionSimNoise#realistic()} when null
     * @return updated field
     */
    public VisionSimulationField noise(VisionSimNoise model) {
        return new VisionSimulationField(targets, model);
    }
}
