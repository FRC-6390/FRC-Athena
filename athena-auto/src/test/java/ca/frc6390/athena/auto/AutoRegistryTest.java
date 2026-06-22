package ca.frc6390.athena.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.commands.CommandSpec;

class AutoRegistryTest {
    @Test
    void registeredSourceCanBeLookedUp() {
        AutoRegistry registry = new AutoRegistry();
        AutoSource source = path -> CommandSpec.create(path).toSpec();

        registry.register("pathplanner", source);

        assertSame(source, registry.require("pathplanner"));
        assertTrue(registry.find("pathplanner").isPresent());
    }

    @Test
    void missingSourceExplainsDependencyGuidance() {
        AutoRegistry registry = new AutoRegistry();

        MissingAutoSourceException error = assertThrows(
                MissingAutoSourceException.class,
                () -> registry.require("choreo"));

        assertTrue(error.getMessage().contains("choreo"));
        assertTrue(error.getMessage().contains("athena-* auto module"));
    }

    @Test
    void sourceCanLoadCommandDescriptor() {
        AutoRegistry registry = new AutoRegistry()
                .register("sim", path -> CommandSpec.create("auto:" + path).toSpec());

        CommandSpec command = registry.require("sim").load("leave");

        assertEquals("auto:leave", command.name());
    }
}
