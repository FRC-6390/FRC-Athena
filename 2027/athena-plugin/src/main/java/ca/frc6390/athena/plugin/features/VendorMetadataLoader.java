package ca.frc6390.athena.plugin.features;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;

import groovy.json.JsonSlurper;

/**
 * Loads vendor adapter metadata from classpath resources.
 */
public final class VendorMetadataLoader {
    /** Resource directory scanned for vendor metadata. */
    public static final String RESOURCE_PREFIX = "META-INF/athena/vendors/";

    private static final String BUILT_IN_CTRE = RESOURCE_PREFIX + "ctre.json";
    private static final String BUILT_IN_REV = RESOURCE_PREFIX + "rev.json";
    private static final String BUILT_IN_PHOTONVISION = RESOURCE_PREFIX + "photonvision.json";
    private static final String BUILT_IN_LIMELIGHT = RESOURCE_PREFIX + "limelight.json";
    private static final String BUILT_IN_PATHPLANNER = RESOURCE_PREFIX + "pathplanner.json";
    private static final String BUILT_IN_CHOREO = RESOURCE_PREFIX + "choreo.json";
    private static final String BUILT_IN_STUDICA = RESOURCE_PREFIX + "studica.json";

    private final ClassLoader classLoader;

    /**
     * Creates a loader.
     *
     * @param classLoader class loader to scan
     */
    public VendorMetadataLoader(ClassLoader classLoader) {
        this.classLoader = Objects.requireNonNull(classLoader, "classLoader");
    }

    /**
     * Loads the built-in metadata resources shipped by Athena.
     *
     * @return built-in vendor features
     */
    public List<VendorFeature> loadBuiltIns() {
        return loadResources(List.of(
                BUILT_IN_CTRE,
                BUILT_IN_REV,
                BUILT_IN_PHOTONVISION,
                BUILT_IN_LIMELIGHT,
                BUILT_IN_PATHPLANNER,
                BUILT_IN_CHOREO,
                BUILT_IN_STUDICA));
    }

    /**
     * Loads known resource paths and rejects conflicting duplicate feature keys.
     *
     * @param resourcePaths resource paths
     * @return vendor features in resource order
     */
    public List<VendorFeature> loadResources(List<String> resourcePaths) {
        Map<String, VendorFeature> features = new LinkedHashMap<>();
        for (String resourcePath : resourcePaths) {
            loadSingleResource(resourcePath, features);
        }
        return List.copyOf(features.values());
    }

    /**
     * Parses one metadata resource.
     *
     * @param reader metadata reader
     * @param source source name for diagnostics
     * @return parsed vendor feature
     */
    public VendorFeature parse(Reader reader, String source) {
        Object value = new JsonSlurper().parse(reader);
        if (!(value instanceof Map<?, ?> map)) {
            throw new VendorMetadataException(source + " must contain a JSON object.");
        }

        String feature = requiredString(map, "feature", source).toLowerCase(Locale.ROOT);
        String displayName = optionalString(map, "displayName", feature);
        Map<?, ?> detect = requiredMap(map, "detect", source);
        List<String> vendordepUuids = stringList(detect.get("vendordepUuids"), source, "detect.vendordepUuids");
        List<String> dependencies = stringList(detect.get("dependencies"), source, "detect.dependencies");
        List<String> artifacts = stringList(map.get("artifacts"), source, "artifacts");

        if (artifacts.isEmpty()) {
            throw new VendorMetadataException(source + " must declare at least one artifact.");
        }

        return new VendorFeature(feature, displayName, dependencies, vendordepUuids, artifacts);
    }

    private void loadSingleResource(String resourcePath, Map<String, VendorFeature> features) {
        try {
            Enumeration<URL> resources = classLoader.getResources(resourcePath);
            if (!resources.hasMoreElements()) {
                throw new VendorMetadataException("Missing Athena vendor metadata resource " + resourcePath + ".");
            }
            while (resources.hasMoreElements()) {
                URL resource = resources.nextElement();
                try (InputStream input = resource.openStream();
                        Reader reader = new InputStreamReader(input, StandardCharsets.UTF_8)) {
                    merge(features, parse(reader, resource.toExternalForm()), resource.toExternalForm());
                }
            }
        } catch (IOException exception) {
            throw new VendorMetadataException("Failed to load Athena vendor metadata " + resourcePath + ".", exception);
        }
    }

    private void merge(Map<String, VendorFeature> features, VendorFeature candidate, String source) {
        VendorFeature existing = features.putIfAbsent(candidate.name(), candidate);
        if (existing != null && !existing.equals(candidate)) {
            throw new VendorMetadataException("Conflicting Athena vendor metadata for feature "
                    + candidate.name() + " from " + source + ".");
        }
    }

    private String requiredString(Map<?, ?> map, String key, String source) {
        Object value = map.get(key);
        if (value instanceof String text && !text.isBlank()) {
            return text.trim();
        }
        throw new VendorMetadataException(source + " must declare string field " + key + ".");
    }

    private String optionalString(Map<?, ?> map, String key, String defaultValue) {
        Object value = map.get(key);
        if (value instanceof String text && !text.isBlank()) {
            return text.trim();
        }
        return defaultValue;
    }

    private Map<?, ?> requiredMap(Map<?, ?> map, String key, String source) {
        Object value = map.get(key);
        if (value instanceof Map<?, ?> child) {
            return child;
        }
        throw new VendorMetadataException(source + " must declare object field " + key + ".");
    }

    private List<String> stringList(Object value, String source, String field) {
        if (value == null) {
            return List.of();
        }
        if (!(value instanceof List<?> values)) {
            throw new VendorMetadataException(source + " field " + field + " must be an array.");
        }
        List<String> result = new ArrayList<>();
        for (Object item : values) {
            if (!(item instanceof String text) || text.isBlank()) {
                throw new VendorMetadataException(source + " field " + field + " must contain only nonblank strings.");
            }
            result.add(text.trim());
        }
        return result;
    }
}
