package ca.frc6390.athena.plugin.statespec;

import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.artifacts.Configuration;
import org.gradle.api.tasks.compile.JavaCompile;

import java.io.File;
import java.net.URISyntaxException;
import java.security.CodeSource;
import java.util.ArrayList;
import java.util.List;

public class AthenaPluginGradlePlugin implements Plugin<Project> {
    private static final String JAVAC_PLUGIN_FLAG = "-Xplugin:AthenaStateDsl";
    private static final List<String> JAVAC_EXPORTS = List.of(
            "--add-exports=jdk.compiler/com.sun.tools.javac.api=ALL-UNNAMED",
            "--add-exports=jdk.compiler/com.sun.tools.javac.code=ALL-UNNAMED",
            "--add-exports=jdk.compiler/com.sun.tools.javac.tree=ALL-UNNAMED",
            "--add-exports=jdk.compiler/com.sun.tools.javac.util=ALL-UNNAMED"
    );

    @Override
    public void apply(Project project) {
        AthenaPluginExtension extension = project.getExtensions()
                .create("athenaPlugin", AthenaPluginExtension.class);

        project.getPlugins().withId("java", ignored -> {
            project.afterEvaluate(p -> {
                if (!extension.isEnabled()) {
                    return;
                }
                wireSelfArtifact(project, extension);
                configureJavaCompileTasks(project);
            });
        });
    }

    private void wireSelfArtifact(Project project, AthenaPluginExtension extension) {
        if (!extension.isSelfArtifactClasspathEnabled()) {
            return;
        }
        File self = resolveSelfArtifact();
        if (self == null || !self.exists()) {
            project.getLogger().warn("Athena plugin could not resolve its own artifact for compile classpath wiring.");
            return;
        }

        addFileDependency(project, "compileOnly", self);
        addFileDependency(project, "annotationProcessor", self);
        addFileDependency(project, "testCompileOnly", self);
        addFileDependency(project, "testAnnotationProcessor", self);
    }

    private void addFileDependency(Project project, String configurationName, File artifact) {
        Configuration cfg = project.getConfigurations().findByName(configurationName);
        if (cfg == null) {
            return;
        }
        project.getDependencies().add(configurationName, project.files(artifact));
    }

    private void configureJavaCompileTasks(Project project) {
        project.getTasks().withType(JavaCompile.class).configureEach(task -> {
            if (!task.getOptions().getCompilerArgs().contains(JAVAC_PLUGIN_FLAG)) {
                task.getOptions().getCompilerArgs().add(JAVAC_PLUGIN_FLAG);
            }

            task.getOptions().setFork(true);
            List<String> jvmArgs = new ArrayList<>();
            if (task.getOptions().getForkOptions().getJvmArgs() != null) {
                jvmArgs.addAll(task.getOptions().getForkOptions().getJvmArgs());
            }
            for (String arg : JAVAC_EXPORTS) {
                if (!jvmArgs.contains(arg)) {
                    jvmArgs.add(arg);
                }
            }
            task.getOptions().getForkOptions().setJvmArgs(jvmArgs);
        });
    }

    private File resolveSelfArtifact() {
        try {
            CodeSource source = AthenaPluginGradlePlugin.class.getProtectionDomain().getCodeSource();
            if (source == null || source.getLocation() == null) {
                return null;
            }
            return new File(source.getLocation().toURI());
        } catch (URISyntaxException ex) {
            return null;
        }
    }
}
