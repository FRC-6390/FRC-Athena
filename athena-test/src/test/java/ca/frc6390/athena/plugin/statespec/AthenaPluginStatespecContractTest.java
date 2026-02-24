package ca.frc6390.athena.plugin.statespec;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.gradle.api.Project;
import org.gradle.api.tasks.compile.JavaCompile;
import org.gradle.testfixtures.ProjectBuilder;
import org.junit.jupiter.api.Test;

final class AthenaPluginStatespecContractTest {

    @Test
    void extensionDefaultsAndMutatorsAreStable() {
        AthenaPluginExtension ext = new AthenaPluginExtension();

        assertTrue(ext.isEnabled());
        assertTrue(ext.isSelfArtifactClasspathEnabled());

        ext.setEnabled(false);
        ext.setSelfArtifactClasspathEnabled(false);

        assertFalse(ext.isEnabled());
        assertFalse(ext.isSelfArtifactClasspathEnabled());
    }

    @Test
    void javacPluginExposesExpectedName() {
        AthenaStateDslJavacPlugin plugin = new AthenaStateDslJavacPlugin();
        assertEquals("AthenaStateDsl", plugin.getName());
    }

    @Test
    void gradlePluginRegistersExtensionAndCanApplyToJavaProject() {
        Project project = ProjectBuilder.builder().build();
        project.getPluginManager().apply("java");

        AthenaPluginGradlePlugin plugin = new AthenaPluginGradlePlugin();
        plugin.apply(project);

        AthenaPluginExtension ext = project.getExtensions().findByType(AthenaPluginExtension.class);
        assertNotNull(ext);

        JavaCompile compileJava = (JavaCompile) project.getTasks().getByName("compileJava");
        assertNotNull(compileJava);
    }
}
