package ca.frc6390.athena.plugin.features;

import java.util.List;

/**
 * Result of Athena feature selection.
 *
 * @param athenaDependencies selected Athena feature coordinates
 * @param vendorDependencies selected vendor adapter coordinates
 */
public record FeatureSelection(List<String> athenaDependencies, List<String> vendorDependencies) {
    public FeatureSelection {
        athenaDependencies = List.copyOf(athenaDependencies);
        vendorDependencies = List.copyOf(vendorDependencies);
    }

    /**
     * Returns all selected dependencies in application order.
     *
     * @return all dependency coordinates
     */
    public List<String> allDependencies() {
        return java.util.stream.Stream.concat(athenaDependencies.stream(), vendorDependencies.stream()).toList();
    }
}
