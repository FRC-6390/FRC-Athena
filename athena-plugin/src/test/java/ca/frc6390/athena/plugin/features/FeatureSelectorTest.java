package ca.frc6390.athena.plugin.features;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class FeatureSelectorTest {
    @Test
    void defaultModulesAreSelectedByArtifactName() {
        FeatureSelection selection = FeatureSelector.builtIn("ca.frc6390.athena", "2027.0.0").select(FeatureRequest.empty());

        assertTrue(selection.athenaDependencies().contains("ca.frc6390.athena:athena-api:2027.0.0"));
        assertTrue(selection.athenaDependencies().contains("ca.frc6390.athena:athena-runtime:2027.0.0"));
        assertFalse(selection.athenaDependencies().contains("ca.frc6390.athena:athena-plugin:2027.0.0"));
        assertFalse(selection.athenaDependencies().contains("ca.frc6390.athena:athena-telemetry:2027.0.0"));
    }

    @Test
    void explicitModulesNormalizeNames() {
        FeatureRequest request = new FeatureRequest(Set.of("DriveTrain", "wpilib"), Set.of(), Set.of(), Set.of());

        FeatureSelection selection = FeatureSelector.builtIn("g", "v").select(request);

        assertTrue(selection.athenaDependencies().contains("g:athena-drivetrain:v"));
        assertTrue(selection.athenaDependencies().contains("g:athena-wpilib:v"));
    }

    @Test
    void unknownModulesFailWithUsefulName() {
        FeatureRequest request = new FeatureRequest(Set.of("not-a-module"), Set.of(), Set.of(), Set.of());

        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> FeatureSelector.builtIn("g", "v").select(request));
        assertEquals("Unknown Athena module 'not-a-module'.", error.getMessage());
    }

    @Test
    void vendorsCanBeDetectedFromCoordinates() {
        VendorFeature vendor = new VendorFeature(
                "demo",
                List.of("detected:lib"),
                List.of("uuid"),
                "vendor:runtime");
        FeatureRequest request = new FeatureRequest(Set.of(), Set.of(), Set.of("detected:lib"), Set.of());

        FeatureSelection selection = new FeatureSelector("g", "v", List.of(vendor)).select(request);

        assertEquals(List.of("vendor:runtime:v"), selection.vendorDependencies());
    }
}
