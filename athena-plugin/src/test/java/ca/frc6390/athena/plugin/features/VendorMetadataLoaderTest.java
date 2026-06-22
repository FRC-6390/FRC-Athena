package ca.frc6390.athena.plugin.features;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.StringReader;
import java.util.List;

import org.junit.jupiter.api.Test;

class VendorMetadataLoaderTest {
    private final VendorMetadataLoader loader = new VendorMetadataLoader(getClass().getClassLoader());

    @Test
    void loadsBuiltInVendorMetadataFromResources() {
        List<VendorFeature> vendors = loader.loadBuiltIns();

        assertTrue(vendors.stream().anyMatch(vendor -> vendor.name().equals("ctre")));
        assertTrue(vendors.stream().anyMatch(vendor -> vendor.name().equals("rev")));
        assertTrue(vendors.stream().anyMatch(vendor -> vendor.name().equals("photonvision")));
        assertTrue(vendors.stream().anyMatch(vendor -> vendor.name().equals("limelight")));
        assertTrue(vendors.stream().anyMatch(vendor -> vendor.name().equals("pathplanner")));
        assertTrue(vendors.stream().anyMatch(vendor -> vendor.name().equals("choreo")));
        assertTrue(vendors.stream().anyMatch(vendor -> vendor.name().equals("studica")));
        assertTrue(vendors.stream()
                .flatMap(vendor -> vendor.athenaArtifacts().stream())
                .anyMatch("ca.frc6390.athena:athena-vendor-ctre"::equals));
        assertTrue(vendors.stream()
                .flatMap(vendor -> vendor.athenaArtifacts().stream())
                .anyMatch("ca.frc6390.athena:athena-vendor-photonvision"::equals));
        assertTrue(vendors.stream()
                .flatMap(vendor -> vendor.athenaArtifacts().stream())
                .anyMatch("ca.frc6390.athena:athena-vendor-pathplanner"::equals));
        assertTrue(vendors.stream()
                .flatMap(vendor -> vendor.athenaArtifacts().stream())
                .anyMatch("ca.frc6390.athena:athena-vendor-studica"::equals));
    }

    @Test
    void parsesThirdPartyVendorMetadata() {
        VendorFeature vendor = loader.parse(new StringReader("""
                {
                  "feature": "acme",
                  "displayName": "Acme Robotics",
                  "detect": {
                    "vendordepUuids": ["aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"],
                    "dependencies": ["com.acme.frc:AcmeLib-java"]
                  },
                  "artifacts": [
                    "com.acme.frc:athena-vendor-acme",
                    "com.acme.frc:athena-vendor-acme-extra"
                  ]
                }
                """), "acme.json");

        assertEquals("acme", vendor.name());
        assertEquals("Acme Robotics", vendor.displayName());
        assertEquals(List.of("com.acme.frc:AcmeLib-java"), vendor.dependencyCoordinates());
        assertEquals(List.of("aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"), vendor.vendordepUuids());
        assertEquals(List.of(
                "com.acme.frc:athena-vendor-acme:2027.0.0",
                "com.acme.frc:athena-vendor-acme-extra:2027.0.0"), vendor.artifactsWithVersion("2027.0.0"));
    }

    @Test
    void rejectsMetadataWithoutArtifacts() {
        VendorMetadataException exception = assertThrows(VendorMetadataException.class, () -> loader.parse(
                new StringReader("""
                        {
                          "feature": "empty",
                          "detect": {
                            "vendordepUuids": [],
                            "dependencies": []
                          },
                          "artifacts": []
                        }
                        """),
                "empty.json"));

        assertTrue(exception.getMessage().contains("at least one artifact"));
    }
}
