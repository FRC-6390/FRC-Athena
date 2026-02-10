package ca.frc6390.athena.mechanisms.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class MechanismConfigLoaderTest {

    @TempDir
    Path tempDir;

    @Test
    void mergesNamedProfileArraysByName() throws Exception {
        Path base = tempDir.resolve("base.toml");
        Path overlay = tempDir.resolve("overlay.toml");

        Files.writeString(base, """
                name = "Turret"
                mechanism_type = "stateful_turret"

                [control]
                output = "voltage"
                pid_profile = "pos"

                [[control.pid_profiles]]
                name = "pos"
                k_p = 1.0
                k_i = 0.0
                k_d = 0.0
                """);

        Files.writeString(overlay, """
                [control]
                pid_profile = "pos"

                [[control.pid_profiles]]
                name = "pos"
                k_p = 2.0

                [[control.pid_profiles]]
                name = "alt"
                k_p = 3.0
                """);

        MechanismConfigFile merged = MechanismConfigLoader.loadMerged(base, overlay);
        assertNotNull(merged);
        assertNotNull(merged.control());
        assertNotNull(merged.control().pidProfiles());
        // pos gets overridden, alt gets added.
        assertEquals(2, merged.control().pidProfiles().size());
        MechanismPidConfig pos = merged.control().pidProfiles().stream()
                .filter(p -> "pos".equals(p.name()))
                .findFirst()
                .orElseThrow();
        assertEquals(2.0, pos.kP());
    }
}

