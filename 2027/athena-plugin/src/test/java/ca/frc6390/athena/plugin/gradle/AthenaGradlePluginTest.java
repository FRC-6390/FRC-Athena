package ca.frc6390.athena.plugin.gradle;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.gradle.api.Project;
import org.gradle.testfixtures.ProjectBuilder;
import org.junit.jupiter.api.Test;

import ca.frc6390.athena.plugin.features.AthenaFeature;

class AthenaGradlePluginTest {
    @Test
    void pluginAddsDefaultAthenaDependencies() {
        Project project = projectWithJava();

        project.getPluginManager().apply(AthenaGradlePlugin.class);
        evaluate(project);

        List<String> dependencies = implementationCoordinates(project);
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-api:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-mechanisms:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-plugin:2027.0.0-SNAPSHOT"));
    }

    @Test
    void extensionCanEnableOptionalFeatures() {
        Project project = projectWithJava();

        project.getPluginManager().apply(AthenaGradlePlugin.class);
        project.getExtensions().configure(AthenaExtension.class, extension -> {
            extension.features(AthenaFeature.SIMULATION);
            extension.features(AthenaFeature.LOCALIZATION);
            extension.features("vision", "auto", "wpilib", "dashboard");
        });
        evaluate(project);

        List<String> dependencies = implementationCoordinates(project);
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-simulation:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-vision:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-auto:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-localization:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-wpilib:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-dashboard:2027.0.0-SNAPSHOT"));
    }

    @Test
    void extensionCanEnableExplicitVendor() {
        Project project = projectWithJava();

        project.getPluginManager().apply(AthenaGradlePlugin.class);
        project.getExtensions().configure(AthenaExtension.class, extension ->
                extension.vendors("ctre", "photonvision", "limelight", "pathplanner", "choreo"));
        evaluate(project);

        List<String> dependencies = implementationCoordinates(project);
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-vendor-ctre:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-vendor-photonvision:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-vendor-limelight:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-vendor-pathplanner:2027.0.0-SNAPSHOT"));
        assertTrue(dependencies.contains("ca.frc6390.athena:athena-vendor-choreo:2027.0.0-SNAPSHOT"));
    }

    @Test
    void pluginDetectsVendorDependencyCoordinate() {
        Project project = projectWithJava();
        project.getDependencies().add("implementation", "com.revrobotics.frc:REVLib-java:2027.0.0");

        project.getPluginManager().apply(AthenaGradlePlugin.class);
        evaluate(project);

        assertTrue(implementationCoordinates(project).contains(
                "ca.frc6390.athena:athena-vendor-rev:2027.0.0-SNAPSHOT"));
    }

    private Project projectWithJava() {
        Project project = ProjectBuilder.builder().build();
        project.getPluginManager().apply("java");
        return project;
    }

    private void evaluate(Project project) {
        ((org.gradle.api.internal.project.ProjectInternal) project).evaluate();
    }

    private List<String> implementationCoordinates(Project project) {
        return project.getConfigurations()
                .getByName("implementation")
                .getDependencies()
                .stream()
                .map(dependency -> dependency.getGroup() + ":" + dependency.getName() + ":" + dependency.getVersion())
                .toList();
    }
}
