package ca.frc6390.athena.wpilib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class PropertiesStateStoreTest {
    @TempDir Path temporaryDirectory;

    @Test
    void atomicallyPersistsReloadsAndDeletesOriginalState() {
        PropertiesStateStore store = new PropertiesStateStore(temporaryDirectory.resolve("system-state.properties"));
        SystemTuningState expected = new SystemTuningState("0", "100", "60", true, true, false);

        assertTrue(store.save(expected).success());
        assertEquals(expected, store.load().orElseThrow());
        assertTrue(store.delete().success());
        assertFalse(store.load().isPresent());
    }
}
