package ca.frc6390.athena.plugin.gradle;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.gradle.testkit.runner.GradleRunner;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class AthenaGradlePluginTest {
    @TempDir
    private Path projectDir;

    @Test
    void pluginAddsSelectedAthenaAndVendorDependencies() throws IOException {
        Files.writeString(projectDir.resolve("settings.gradle"), "rootProject.name = 'athena-plugin-test'\n");
        Files.writeString(projectDir.resolve("build.gradle"), """
                plugins {
                    id 'java'
                    id 'ca.frc6390.athena'
                }

                athena {
                    setGroup('test.group')
                    setVersion('1.2.3')
                    setAutoDetectVendors(false)
                    features 'vision'
                    vendors 'pathplanner'
                }

                tasks.register('printImplementationDependencies') {
                    doLast {
                        configurations.implementation.dependencies.each {
                            println "${it.group}:${it.name}:${it.version}"
                        }
                    }
                }
                """);

        String output = GradleRunner.create()
                .withProjectDir(projectDir.toFile())
                .withPluginClasspath()
                .withArguments("printImplementationDependencies", "--stacktrace")
                .build()
                .getOutput();

        assertTrue(output.contains("test.group:athena-vision:1.2.3"));
        assertTrue(output.contains("ca.frc6390.athena:athena-vendor-pathplanner:1.2.3"));
        assertTrue(output.contains("BUILD SUCCESSFUL"));
    }

    @Test
    void pluginFailsForUnknownFeatureRequests() throws IOException {
        Files.writeString(projectDir.resolve("settings.gradle"), "rootProject.name = 'athena-plugin-test'\n");
        Files.writeString(projectDir.resolve("build.gradle"), """
                plugins {
                    id 'java'
                    id 'ca.frc6390.athena'
                }

                athena {
                    features 'missing-feature'
                }
                """);

        var result = GradleRunner.create()
                .withProjectDir(projectDir.toFile())
                .withPluginClasspath()
                .withArguments("tasks", "--stacktrace")
                .buildAndFail();

        assertTrue(result.getOutput().contains("Unknown Athena module 'missing-feature'."));
    }
}
