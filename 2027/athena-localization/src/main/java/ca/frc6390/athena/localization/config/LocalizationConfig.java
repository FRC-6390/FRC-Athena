package ca.frc6390.athena.localization.config;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;

import ca.frc6390.athena.localization.spec.FieldBoundsSpec;
import ca.frc6390.athena.localization.spec.LocalizationSpec;
import ca.frc6390.athena.localization.spec.PoseAliasSpec;
import ca.frc6390.athena.localization.spec.SlipDetectionSpec;
import ca.frc6390.athena.localization.spec.VisionWeightSpec;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Student-facing localization declaration.
 */
public final class LocalizationConfig {
    private final String name;
    private final VisionWeightConfig vision = new VisionWeightConfig();
    private SlipDetectionSpec slipDetection = SlipDetectionSpec.enabled(0.18, 0.35);
    private FieldBoundsSpec fieldBounds = FieldBoundsSpec.unbounded();
    private final Map<String, FieldBoundsSpec> fieldBoundsRegions = new LinkedHashMap<>();
    private final Map<String, PoseAliasSpec> aliases = new LinkedHashMap<>();

    private LocalizationConfig(String name) {
        this.name = name == null || name.isBlank() ? "localization" : name;
    }

    /**
     * Creates a localization declaration.
     *
     * @param name localization name
     * @return localization config
     */
    public static LocalizationConfig create(String name) {
        return new LocalizationConfig(name);
    }

    /**
     * Configures vision measurement weighting.
     *
     * @param customizer vision weighting customizer
     * @return this config
     */
    public LocalizationConfig vision(Consumer<VisionWeightConfig> customizer) {
        if (customizer != null) {
            customizer.accept(vision);
        }
        return this;
    }

    /**
     * Configures slip detection thresholds.
     *
     * @param customizer slip detection customizer
     * @return this config
     */
    public LocalizationConfig slip(Consumer<SlipDetectionConfig> customizer) {
        SlipDetectionConfig config = SlipDetectionConfig.from(slipDetection);
        if (customizer != null) {
            customizer.accept(config);
        }
        slipDetection = config.toSpec();
        return this;
    }

    /**
     * Sets rectangular field bounds for accepted poses.
     *
     * @param name bounds name
     * @param minX minimum x in meters
     * @param minY minimum y in meters
     * @param maxX maximum x in meters
     * @param maxY maximum y in meters
     * @return this config
     */
    public LocalizationConfig fieldBounds(String name, double minX, double minY, double maxX, double maxY) {
        FieldBoundsSpec bounds = new FieldBoundsSpec(name, minX, minY, maxX, maxY);
        fieldBounds = bounds;
        fieldBoundsRegions.put(bounds.name(), bounds);
        return this;
    }

    /**
     * Adds or replaces a named autonomous pose alias.
     *
     * @param name alias name
     * @param pose pose snapshot
     * @return this config
     */
    public LocalizationConfig poseAlias(String name, PoseSnapshot pose) {
        String aliasName = Objects.requireNonNull(name, "name");
        aliases.put(aliasName, new PoseAliasSpec(aliasName, Objects.requireNonNull(pose, "pose")));
        return this;
    }

    /**
     * Adds or replaces a named autonomous pose alias.
     *
     * @param name alias name
     * @param xMeters field x position
     * @param yMeters field y position
     * @param headingRadians heading
     * @return this config
     */
    public LocalizationConfig poseAlias(String name, double xMeters, double yMeters, double headingRadians) {
        return poseAlias(name, new PoseSnapshot(xMeters, yMeters, headingRadians));
    }

    /**
     * Lowers this declaration into an immutable spec.
     *
     * @return localization spec
     */
    public LocalizationSpec toSpec() {
        return new LocalizationSpec(
                name,
                vision.toSpec(),
                slipDetection,
                fieldBounds,
                fieldBoundsRegions.values().stream().toList(),
                aliases.values().stream().toList());
    }
}
