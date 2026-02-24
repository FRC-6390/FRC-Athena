package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.AutoRegistryExamples;
import java.util.NoSuchElementException;
import java.util.UUID;
import org.junit.jupiter.api.Test;

final class AutoRegistryExamplesTest {

    @Test
    void registryAccessorReturnsSingleton() {
        assertNotNull(AutoRegistryExamples.registry());
    }

    @Test
    void registeredEngineCanBeResolved() {
        String key = "test.custom." + UUID.randomUUID();

        AutoRegistryExamples.registerCustomEngine(key);

        assertEquals(RobotAuto.AutoSource.CUSTOM, AutoRegistryExamples.requireEngine(key));
    }

    @Test
    void missingEngineThrowsClearMessage() {
        String key = "missing." + UUID.randomUUID();

        NoSuchElementException ex =
                assertThrows(NoSuchElementException.class, () -> AutoRegistryExamples.requireEngine(key));

        assertTrue(ex.getMessage().contains("Missing provider for auto engine '" + key + "'"));
    }
}
