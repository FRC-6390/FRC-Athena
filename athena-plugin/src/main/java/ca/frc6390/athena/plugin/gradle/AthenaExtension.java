package ca.frc6390.athena.plugin.gradle;

import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;

import ca.frc6390.athena.plugin.features.AthenaFeature;

/**
 * Gradle extension for Athena dependency selection.
 */
public class AthenaExtension {
    private String group = "ca.frc6390.athena";
    private String version = "2027.0.0-SNAPSHOT";
    private boolean autoDetectVendors = true;
    private final Set<AthenaFeature> features = new LinkedHashSet<>();
    private final Set<String> vendors = new LinkedHashSet<>();
    private final Set<String> vendordepUuids = new LinkedHashSet<>();

    /**
     * Returns Athena Maven group.
     *
     * @return group id
     */
    public String group() {
        return group;
    }

    /**
     * Sets Athena Maven group.
     *
     * @param group group id
     */
    public void setGroup(String group) {
        this.group = group == null || group.isBlank() ? this.group : group;
    }

    /**
     * Returns Athena version.
     *
     * @return version
     */
    public String version() {
        return version;
    }

    /**
     * Sets Athena version.
     *
     * @param version version
     */
    public void setVersion(String version) {
        this.version = version == null || version.isBlank() ? this.version : version;
    }

    /**
     * Returns whether vendor dependencies should be detected.
     *
     * @return true when auto-detection is enabled
     */
    public boolean autoDetectVendors() {
        return autoDetectVendors;
    }

    /**
     * Sets whether vendor dependencies should be detected.
     *
     * @param autoDetectVendors true to detect vendors
     */
    public void setAutoDetectVendors(boolean autoDetectVendors) {
        this.autoDetectVendors = autoDetectVendors;
    }

    /**
     * Enables optional Athena features by enum.
     *
     * @param requestedFeatures requested features
     */
    public void features(AthenaFeature... requestedFeatures) {
        if (requestedFeatures == null) {
            return;
        }
        for (AthenaFeature feature : requestedFeatures) {
            if (feature != null) {
                features.add(feature);
            }
        }
    }

    /**
     * Enables optional Athena features by lower-case name.
     *
     * @param requestedFeatures feature names
     */
    public void features(String... requestedFeatures) {
        if (requestedFeatures == null) {
            return;
        }
        for (String feature : requestedFeatures) {
            if (feature != null && !feature.isBlank()) {
                features.add(AthenaFeature.valueOf(feature.trim().replace('-', '_').toUpperCase(Locale.ROOT)));
            }
        }
    }

    /**
     * Enables vendor adapters by name.
     *
     * @param requestedVendors vendor names
     */
    public void vendors(String... requestedVendors) {
        if (requestedVendors == null) {
            return;
        }
        for (String vendor : requestedVendors) {
            if (vendor != null && !vendor.isBlank()) {
                vendors.add(vendor.trim().toLowerCase(Locale.ROOT));
            }
        }
    }

    /**
     * Adds detected vendordep UUIDs.
     *
     * @param uuids vendordep UUIDs
     */
    public void vendordepUuids(String... uuids) {
        if (uuids == null) {
            return;
        }
        for (String uuid : uuids) {
            if (uuid != null && !uuid.isBlank()) {
                vendordepUuids.add(uuid.trim());
            }
        }
    }

    Set<AthenaFeature> features() {
        return Set.copyOf(features);
    }

    Set<String> vendors() {
        return Set.copyOf(vendors);
    }

    Set<String> vendordepUuids() {
        return Set.copyOf(vendordepUuids);
    }
}
