package ca.frc6390.athena.plugin.features;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;

class FeatureSelectorTest {
    private final FeatureSelector selector = FeatureSelector.builtIn("ca.frc6390.athena", "2027.0.0-test");

    @Test
    void selectsDefaultStudentFacingArtifacts() {
        FeatureSelection selection = selector.select(FeatureRequest.empty());

        assertTrue(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-api:2027.0.0-test"));
        assertTrue(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-mechanisms:2027.0.0-test"));
        assertTrue(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-plugin:2027.0.0-test"));
        assertFalse(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-simulation:2027.0.0-test"));
    }

    @Test
    void includesExplicitOptionalFeature() {
        FeatureRequest request = new FeatureRequest(
                Set.of(
                        AthenaFeature.SIMULATION,
                        AthenaFeature.VISION,
                        AthenaFeature.AUTO,
                        AthenaFeature.LOCALIZATION,
                        AthenaFeature.WPILIB,
                        AthenaFeature.DASHBOARD),
                Set.of(),
                Set.of(),
                Set.of());

        FeatureSelection selection = selector.select(request);

        assertTrue(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-simulation:2027.0.0-test"));
        assertTrue(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-vision:2027.0.0-test"));
        assertTrue(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-auto:2027.0.0-test"));
        assertTrue(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-localization:2027.0.0-test"));
        assertTrue(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-wpilib:2027.0.0-test"));
        assertTrue(selection.athenaDependencies().contains(
                "ca.frc6390.athena:athena-dashboard:2027.0.0-test"));
    }

    @Test
    void detectsVendorFromDependencyCoordinate() {
        FeatureRequest request = new FeatureRequest(
                Set.of(),
                Set.of(),
                Set.of("com.revrobotics.frc:REVLib-java", "com.studica.frc:Studica-java"),
                Set.of());

        FeatureSelection selection = selector.select(request);

        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-rev:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-studica:2027.0.0-test"));
    }

    @Test
    void detectsVisionVendorsFromDependencyCoordinates() {
        FeatureRequest request = new FeatureRequest(
                Set.of(),
                Set.of(),
                Set.of(
                        "org.photonvision:photonlib-java",
                        "com.limelightvision:limelightlib-java",
                        "com.pathplanner.lib:PathplannerLib-java",
                        "org.choreo:choreo-lib-java"),
                Set.of());

        FeatureSelection selection = selector.select(request);

        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-photonvision:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-limelight:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-pathplanner:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-choreo:2027.0.0-test"));
    }

    @Test
    void detectsVendorFromVendordepUuid() {
        FeatureRequest request = new FeatureRequest(
                Set.of(),
                Set.of(),
                Set.of(),
                Set.of("e995de00-2c64-4df5-8831-c1441420ff19"));

        FeatureSelection selection = selector.select(request);

        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-ctre:2027.0.0-test"));
    }

    @Test
    void includesExplicitVendor() {
        FeatureRequest request = new FeatureRequest(
                Set.of(),
                Set.of("ctre", "photonvision", "limelight", "pathplanner", "choreo", "studica"),
                Set.of(),
                Set.of());

        FeatureSelection selection = selector.select(request);

        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-ctre:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-photonvision:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-limelight:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-pathplanner:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-choreo:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "ca.frc6390.athena:athena-vendor-studica:2027.0.0-test"));
    }

    @Test
    void includesAllArtifactsFromSelectedVendorMetadata() {
        FeatureSelector customSelector = new FeatureSelector(
                "ca.frc6390.athena",
                "2027.0.0-test",
                List.of(new VendorFeature(
                        "acme",
                        "Acme Robotics",
                        List.of("com.acme.frc:AcmeLib-java"),
                        List.of(),
                        List.of(
                                "com.acme.frc:athena-vendor-acme",
                                "com.acme.frc:athena-vendor-acme-extra"))));
        FeatureRequest request = new FeatureRequest(
                Set.of(),
                Set.of(),
                Set.of("com.acme.frc:AcmeLib-java"),
                Set.of());

        FeatureSelection selection = customSelector.select(request);

        assertTrue(selection.vendorDependencies().contains(
                "com.acme.frc:athena-vendor-acme:2027.0.0-test"));
        assertTrue(selection.vendorDependencies().contains(
                "com.acme.frc:athena-vendor-acme-extra:2027.0.0-test"));
    }
}
