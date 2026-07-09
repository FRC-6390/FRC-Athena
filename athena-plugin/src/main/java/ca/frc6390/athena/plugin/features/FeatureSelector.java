package ca.frc6390.athena.plugin.features;

import java.util.ArrayList;
import java.util.Comparator;
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
        Set<ModuleArtifact> selectedModules = new java.util.LinkedHashSet<>();
        for (ModuleArtifact module : ModuleArtifact.values()) {
            if (module.defaultEnabled()) {
                selectedModules.add(module);
            }
        }
        request.explicitModules().stream()
                .map(ModuleArtifact::fromRequestName)
                .forEach(selectedModules::add);

        List<String> athena = selectedModules.stream()
                .sorted(Comparator.comparing(ModuleArtifact::artifactId))
                .map(module -> module.coordinate(group, version))
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
