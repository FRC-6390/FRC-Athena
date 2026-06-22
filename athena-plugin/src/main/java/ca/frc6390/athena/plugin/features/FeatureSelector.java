package ca.frc6390.athena.plugin.features;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

/**
 * Selects Athena artifacts from explicit feature requests and detected vendor
 * dependencies.
 */
public final class FeatureSelector {
    private final String group;
    private final String version;
    private final List<VendorFeature> vendors;

    /**
     * Creates a selector.
     *
     * @param group Athena group id
     * @param version Athena version
     * @param vendors known vendor features
     */
    public FeatureSelector(String group, String version, List<VendorFeature> vendors) {
        this.group = group;
        this.version = version;
        this.vendors = List.copyOf(vendors);
    }

    /**
     * Creates a selector with built-in vendors.
     *
     * @param group Athena group id
     * @param version Athena version
     * @return selector
     */
    public static FeatureSelector builtIn(String group, String version) {
        return new FeatureSelector(group, version, VendorFeature.builtIns());
    }

    /**
     * Selects dependencies for a request.
     *
     * @param request feature request
     * @return selected dependencies
     */
    public FeatureSelection select(FeatureRequest request) {
        Set<AthenaFeature> selectedFeatures = EnumSet.noneOf(AthenaFeature.class);
        for (AthenaFeature feature : AthenaFeature.values()) {
            if (feature.defaultEnabled()) {
                selectedFeatures.add(feature);
            }
        }
        selectedFeatures.addAll(request.explicitFeatures());

        List<String> athena = selectedFeatures.stream()
                .sorted(Comparator.comparing(AthenaFeature::artifactId))
                .map(feature -> feature.coordinate(group, version))
                .toList();

        List<String> vendorArtifacts = new ArrayList<>();
        for (VendorFeature vendor : vendors) {
            if (isSelected(vendor, request)) {
                vendorArtifacts.addAll(vendor.artifactsWithVersion(version));
            }
        }
        vendorArtifacts.sort(Comparator.naturalOrder());

        return new FeatureSelection(athena, vendorArtifacts);
    }

    private boolean isSelected(VendorFeature vendor, FeatureRequest request) {
        if (request.explicitVendors().contains(vendor.name())) {
            return true;
        }
        boolean dependencyPresent = vendor.dependencyCoordinates().stream()
                .anyMatch(request.detectedDependencyCoordinates()::contains);
        boolean vendordepPresent = vendor.vendordepUuids().stream()
                .anyMatch(request.detectedVendordepUuids()::contains);
        return dependencyPresent || vendordepPresent;
    }
}
