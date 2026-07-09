package ca.frc6390.athena.vision.runtime;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class CameraAdaptersTest {
    @TempDir
    Path tempDir;

    @Test
    void discoverySkipsUnavailableServiceProviders() throws IOException {
        CameraAdapters.clearCache();
        Path services = tempDir.resolve("META-INF/services");
        Files.createDirectories(services);
        Files.writeString(
                services.resolve(CameraAdapter.class.getName()),
                "missing.vendor.MissingCameraAdapter\n");

        ClassLoader original = Thread.currentThread().getContextClassLoader();
        try (URLClassLoader loader = new URLClassLoader(
                new URL[] {tempDir.toUri().toURL()},
                original)) {
            Thread.currentThread().setContextClassLoader(loader);

            assertTrue(assertDoesNotThrow(CameraAdapters::discover).isEmpty());
        } finally {
            Thread.currentThread().setContextClassLoader(original);
            CameraAdapters.clearCache();
        }
    }

    @Test
    void discoveryIsCachedBetweenCalls() {
        CameraAdapters.clearCache();

        assertSame(CameraAdapters.discover(), CameraAdapters.discover());

        CameraAdapters.clearCache();
    }
}
