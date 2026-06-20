package ca.frc6390.athena.localization.spec;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import ca.frc6390.athena.runtime.validation.ValidationReport;

/**
 * Immutable localization declaration.
 *
 * @param name localization name
 * @param visionWeight vision measurement weighting
 * @param slipDetection slip detection settings
 * @param fieldBounds accepted field bounds
 * @param fieldBoundsRegions named field bounding boxes
 * @param poseAliases autonomous pose aliases
 */
public record LocalizationSpec(
        String name,
        VisionWeightSpec visionWeight,
        SlipDetectionSpec slipDetection,
        FieldBoundsSpec fieldBounds,
        List<FieldBoundsSpec> fieldBoundsRegions,
        List<PoseAliasSpec> poseAliases) {
    public LocalizationSpec {
        name = name == null || name.isBlank() ? "localization" : name;
        visionWeight = visionWeight == null ? VisionWeightSpec.defaults() : visionWeight;
        slipDetection = slipDetection == null ? SlipDetectionSpec.enabled(0.18, 0.35) : slipDetection;
        fieldBounds = fieldBounds == null ? FieldBoundsSpec.unbounded() : fieldBounds;
        fieldBoundsRegions = fieldBoundsRegions == null ? List.of() : List.copyOf(fieldBoundsRegions);
        poseAliases = poseAliases == null ? List.of() : List.copyOf(poseAliases);
    }

    /**
     * Finds an autonomous pose alias by name.
     *
     * @param aliasName alias name
     * @return matching alias
     */
    public Optional<PoseAliasSpec> findPoseAlias(String aliasName) {
        return poseAliases.stream()
                .filter(alias -> alias.name().equals(aliasName))
                .findFirst();
    }

    /**
     * Finds a named field bounding box.
     *
     * @param boundsName bounds name
     * @return matching bounds
     */
    public Optional<FieldBoundsSpec> findFieldBounds(String boundsName) {
        return fieldBoundsRegions.stream()
                .filter(bounds -> bounds.name().equals(boundsName))
                .findFirst();
    }

    /**
     * Validates localization values.
     *
     * @return validation report
     */
    public ValidationReport validate() {
        ValidationReport.Builder report = ValidationReport.builder();
        if (!visionWeight.isValid()) {
            report.error("localization.invalid-vision-weight", name, "Vision standard deviations must be finite and positive.");
        }
        if (!slipDetection.isValid()) {
            report.error("localization.invalid-slip-detection", name, "Slip thresholds must be finite and non-negative.");
        }
        for (FieldBoundsSpec bounds : fieldBoundsRegions) {
            if (!bounds.isFinite() || !bounds.isOrdered()) {
                report.error("localization.invalid-field-bounds", name + "." + bounds.name(),
                        "Field bounds must be finite and minimums must not exceed maximums.");
            }
        }
        Map<String, PoseAliasSpec> seen = new LinkedHashMap<>();
        for (PoseAliasSpec alias : poseAliases) {
            if (!seen.containsKey(alias.name())) {
                seen.put(alias.name(), alias);
            } else {
                report.error("localization.duplicate-pose-alias", name + "." + alias.name(),
                        "Pose alias names must be unique.");
            }
            if (!alias.isFinite()) {
                report.error("localization.invalid-pose-alias", name + "." + alias.name(),
                        "Pose aliases must contain finite pose values.");
            }
            if (fieldBounds.isFinite() && alias.isFinite() && !fieldBounds.contains(alias.pose())) {
                report.error("localization.pose-out-of-bounds", name + "." + alias.name(),
                        "Pose alias must be inside field bounds.");
            }
        }
        return report.build();
    }
}
