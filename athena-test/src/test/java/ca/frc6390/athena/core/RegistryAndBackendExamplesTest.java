package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.examples.RegistryAndBackendExamples;
import ca.frc6390.athena.core.examples.RegistryAndBackendExamples.ExampleBackend;
import ca.frc6390.athena.core.examples.RegistryAndBackendExamples.ExampleRegistry;
import java.util.NoSuchElementException;
import java.util.List;
import org.junit.jupiter.api.Test;

final class RegistryAndBackendExamplesTest {

    @Test
    void registryStoresAndResolvesValues() {
        ExampleRegistry registry = new ExampleRegistry().add("drive", "swerve");
        assertEquals("swerve", registry.requireValue("drive"));
    }

    @Test
    void registryMissingKeyThrowsConfiguredMessage() {
        ExampleRegistry registry = new ExampleRegistry();
        NoSuchElementException ex =
                assertThrows(NoSuchElementException.class, () -> registry.requireValue("missing"));
        assertTrue(ex.getMessage().contains("Missing key 'missing'"));
    }

    @Test
    void preferredBackendSelectsHighestPrioritySupportingBackend() {
        AutoBackend low = new ExampleBackend(RobotAuto.AutoSource.CHOREO, 2);
        AutoBackend high = new ExampleBackend(RobotAuto.AutoSource.CHOREO, 5);
        AutoBackend wrongSource = new ExampleBackend(RobotAuto.AutoSource.PATH_PLANNER, 100);

        AutoBackend selected = RegistryAndBackendExamples
                .selectPreferredBackend(List.of(low, high, wrongSource), RobotAuto.AutoSource.CHOREO)
                .orElseThrow();

        assertEquals(high, selected);
    }

    @Test
    void preferredBackendEmptyWhenNoBackendSupportsSource() {
        AutoBackend backend = new ExampleBackend(RobotAuto.AutoSource.PATH_PLANNER, 1);

        assertTrue(RegistryAndBackendExamples
                .selectPreferredBackend(List.of(backend), RobotAuto.AutoSource.CHOREO)
                .isEmpty());
        assertTrue(RegistryAndBackendExamples.selectPreferredBackend(List.of(), RobotAuto.AutoSource.CHOREO).isEmpty());
    }

    @Test
    void defaultBackendMethodsAreSafeNoOps() {
        AutoBackend backend = new AutoBackend() {
            @Override
            public boolean supports(RobotAuto.AutoSource source) {
                return false;
            }
        };

        assertFalse(backend.configureHolonomic(null));
        assertTrue(backend.warmupCommand(RobotAuto.AutoSource.CUSTOM).isEmpty());
        assertFalse(backend.registerNamedCommand("id", () -> null));
        assertTrue(backend.buildAuto(RobotAuto.AutoSource.CUSTOM, "ref").isEmpty());
        assertTrue(backend.getAutoPoses(RobotAuto.AutoSource.CUSTOM, "ref").isEmpty());
    }
}
