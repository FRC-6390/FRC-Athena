package ca.frc6390.athena.plugin.features;

import java.util.Set;

/**
 * Inputs used by Athena's Gradle plugin to select dependencies.
 *
 * @param explicitFeatures explicitly requested Athena features
 * @param explicitVendors explicitly requested vendor adapter names
 * @param detectedDependencyCoordinates dependency coordinates already present
 * @param detectedVendordepUuids vendor dependency UUIDs already present
 */
public record FeatureRequest(
        Set<AthenaFeature> explicitFeatures,
        Set<String> explicitVendors,
        Set<String> detectedDependencyCoordinates,
        Set<String> detectedVendordepUuids) {
    /**
     * Creates an empty request.
     *
     * @return empty request
     */
    public static FeatureRequest empty() {
        return new FeatureRequest(Set.of(), Set.of(), Set.of(), Set.of());
    }

    public FeatureRequest {
        explicitFeatures = Set.copyOf(explicitFeatures);
        explicitVendors = Set.copyOf(explicitVendors);
        detectedDependencyCoordinates = Set.copyOf(detectedDependencyCoordinates);
        detectedVendordepUuids = Set.copyOf(detectedVendordepUuids);
    }
}
