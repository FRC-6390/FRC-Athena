package ca.frc6390.athena.plugin.gradle;

import java.util.Set;
import java.util.stream.Collectors;

import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.artifacts.Dependency;

import ca.frc6390.athena.plugin.features.FeatureRequest;
import ca.frc6390.athena.plugin.features.FeatureSelection;
import ca.frc6390.athena.plugin.features.FeatureSelector;

/**
 * Gradle plugin that adds selected Athena feature and vendor artifacts.
 */
public final class AthenaGradlePlugin implements Plugin<Project> {
    @Override
    public void apply(Project project) {
        AthenaExtension extension = project.getExtensions().create("athena", AthenaExtension.class);
        project.afterEvaluate(evaluated -> applyDependencies(evaluated, extension));
    }

    private void applyDependencies(Project project, AthenaExtension extension) {
        Set<String> detectedDependencies = extension.autoDetectVendors()
                ? detectDependencyCoordinates(project)
                : Set.of();
        Set<String> detectedVendordeps = extension.autoDetectVendors()
                ? extension.vendordepUuids()
                : Set.of();
        FeatureRequest request = new FeatureRequest(
                extension.features(),
                extension.vendors(),
                detectedDependencies,
                detectedVendordeps);
        FeatureSelection selection = FeatureSelector.builtIn(extension.group(), extension.version()).select(request);
        selection.allDependencies().forEach(coordinate -> project.getDependencies().add("implementation", coordinate));
    }

    private Set<String> detectDependencyCoordinates(Project project) {
        return project.getConfigurations().stream()
                .flatMap(configuration -> configuration.getDependencies().stream())
                .map(this::coordinateWithoutVersion)
                .filter(coordinate -> !coordinate.isBlank())
                .collect(Collectors.toSet());
    }

    private String coordinateWithoutVersion(Dependency dependency) {
        String group = dependency.getGroup();
        String name = dependency.getName();
        if (group == null || group.isBlank() || name == null || name.isBlank()) {
            return "";
        }
        return group + ":" + name;
    }
}
