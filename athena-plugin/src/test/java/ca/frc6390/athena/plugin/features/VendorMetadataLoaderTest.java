package ca.frc6390.athena.plugin.features;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.io.StringReader;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class VendorMetadataLoaderTest {
    @TempDir
    private Path tempDir;

    @Test
    void parseNormalizesFeatureAndTrimsStringLists() {
        VendorMetadataLoader loader = new VendorMetadataLoader(getClass().getClassLoader());

        VendorFeature feature = loader.parse(new StringReader("""
                {
                  "feature": " Demo ",
                  "displayName": " Demo Vendor ",
                  "detect": {
                    "vendordepUuids": [" uuid "],
                    "dependencies": [" group:artifact "]
                  },
                  "artifacts": [" ca.frc6390.athena:athena-vendor-demo "]
                }
                """), "demo.json");

        assertEquals("demo", feature.name());
        assertEquals("Demo Vendor", feature.displayName());
        assertEquals(List.of("group:artifact"), feature.dependencyCoordinates());
        assertEquals(List.of("uuid"), feature.vendordepUuids());
        assertEquals(List.of("ca.frc6390.athena:athena-vendor-demo"), feature.athenaArtifacts());
    }

    @Test
    void parseRejectsMalformedMetadata() {
        VendorMetadataLoader loader = new VendorMetadataLoader(getClass().getClassLoader());

        VendorMetadataException error = assertThrows(VendorMetadataException.class,
                () -> loader.parse(new StringReader("""
                        {
                          "feature": "demo",
                          "detect": {"vendordepUuids": [], "dependencies": []},
                          "artifacts": []
                        }
                        """), "bad.json"));

        assertEquals("bad.json must declare at least one artifact.", error.getMessage());
    }

    @Test
    void loadResourcesRejectsMissingResources() {
        VendorMetadataLoader loader = new VendorMetadataLoader(getClass().getClassLoader());

        VendorMetadataException error = assertThrows(VendorMetadataException.class,
                () -> loader.loadResources(List.of("META-INF/athena/vendors/missing-test-resource.json")));

        assertTrue(error.getMessage().contains("Missing Athena vendor metadata resource"));
    }

    @Test
    void loadResourcesRejectsConflictingDuplicateFeatureKeys() throws IOException {
        Path firstRoot = Files.createDirectories(tempDir.resolve("first"));
        Path secondRoot = Files.createDirectories(tempDir.resolve("second"));
        writeVendorMetadata(firstRoot, "demo", "ca.frc6390.athena:athena-vendor-demo");
        writeVendorMetadata(secondRoot, "demo", "ca.frc6390.athena:athena-vendor-other");

        try (URLClassLoader classLoader = new URLClassLoader(new URL[] {
                firstRoot.toUri().toURL(),
                secondRoot.toUri().toURL()
        }, null)) {
            VendorMetadataLoader loader = new VendorMetadataLoader(classLoader);

            VendorMetadataException error = assertThrows(VendorMetadataException.class,
                    () -> loader.loadResources(List.of(VendorMetadataLoader.RESOURCE_PREFIX + "demo.json")));

            assertTrue(error.getMessage().contains("Conflicting Athena vendor metadata for feature demo"));
        }
    }

    @Test
    void builtInResourcesIncludePathPlannerAndChoreo() {
        List<String> names = new VendorMetadataLoader(getClass().getClassLoader()).loadBuiltIns().stream()
                .map(VendorFeature::name)
                .toList();

        assertTrue(names.contains("pathplanner"));
        assertTrue(names.contains("choreo"));
    }

    private static void writeVendorMetadata(Path root, String feature, String artifact) throws IOException {
        Path directory = Files.createDirectories(root.resolve(VendorMetadataLoader.RESOURCE_PREFIX));
        Files.writeString(directory.resolve(feature + ".json"), """
                {
                  "feature": "%s",
                  "detect": {"vendordepUuids": [], "dependencies": []},
                  "artifacts": ["%s"]
                }
                """.formatted(feature, artifact));
    }
}
