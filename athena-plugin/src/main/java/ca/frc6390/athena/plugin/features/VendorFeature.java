package ca.frc6390.athena.plugin.features;

import java.util.List;
import java.util.Objects;

/**
 * Metadata used by the Athena Gradle plugin to select vendor adapter artifacts.
 *
 * @param name short feature name
 * @param displayName human-readable vendor name
 * @param dependencyCoordinates vendor dependency coordinates that imply support
 * @param vendordepUuids vendor dependency UUIDs that imply support
 * @param athenaArtifacts Athena adapter artifact coordinates without versions
 */
public record VendorFeature(
        String name,
        String displayName,
        List<String> dependencyCoordinates,
        List<String> vendordepUuids,
        List<String> athenaArtifacts) {
    public VendorFeature {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(displayName, "displayName");
        dependencyCoordinates = List.copyOf(dependencyCoordinates);
        vendordepUuids = List.copyOf(vendordepUuids);
        athenaArtifacts = List.copyOf(athenaArtifacts);
        if (athenaArtifacts.isEmpty()) {
            throw new IllegalArgumentException("athenaArtifacts must not be empty");
        }
    }

    /**
     * Creates single-artifact metadata.
     *
     * @param name short feature name
     * @param dependencyCoordinates vendor dependency coordinates that imply support
     * @param vendordepUuids vendor dependency UUIDs that imply support
     * @param athenaArtifact Athena adapter artifact coordinate without version
     */
    public VendorFeature(
            String name,
            List<String> dependencyCoordinates,
            List<String> vendordepUuids,
            String athenaArtifact) {
        this(name, name, dependencyCoordinates, vendordepUuids, List.of(athenaArtifact));
    }

    /**
     * Returns built-in vendor feature metadata loaded from classpath resources.
     *
     * @return built-in features
     */
    public static List<VendorFeature> builtIns() {
        return new VendorMetadataLoader(VendorFeature.class.getClassLoader()).loadBuiltIns();
    }

    /**
     * Returns the adapter dependency coordinates for a version.
     *
     * @param version Athena version
     * @return full artifact coordinates
     */
    public List<String> artifactsWithVersion(String version) {
        return athenaArtifacts.stream()
                .map(artifact -> artifact + ":" + version)
                .toList();
    }
}
