package ca.frc6390.athena.hardware.backend;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import ca.frc6390.athena.api.hardware.MotorKinds;

class BackendRegistryTest {
    @TempDir
    Path tempDir;

    @Test
    void discoverySkipsUnavailableServiceProviders() throws IOException {
        Path services = tempDir.resolve("META-INF/services");
        Files.createDirectories(services);
        Files.writeString(
                services.resolve(MotorBackend.class.getName()),
                "missing.vendor.MissingMotorBackend\n");

        ClassLoader original = Thread.currentThread().getContextClassLoader();
        try (URLClassLoader loader = new URLClassLoader(
                new URL[] {tempDir.toUri().toURL()},
                original)) {
            Thread.currentThread().setContextClassLoader(loader);

            BackendRegistry registry = assertDoesNotThrow(BackendRegistry::discover);

            assertTrue(registry.motorBackendFor(MotorKinds.KRAKEN_X60).isEmpty());
        } finally {
            Thread.currentThread().setContextClassLoader(original);
        }
    }
}
