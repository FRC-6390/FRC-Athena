package ca.frc6390.athena.wpilib.system;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.Duration;
import java.util.List;
import org.junit.jupiter.api.Test;

class ClassPreloaderTest {
    @Test
    void findsAthenaClassesOnTheClassPathWithoutTestOrModuleEntries() {
        List<String> names = ClassPreloader.classNames(System.getProperty("java.class.path"), "ca.frc6390.athena.");
        assertTrue(names.contains("ca.frc6390.athena.wpilib.system.ClassPreloader"));
        assertTrue(names.contains("ca.frc6390.athena.mechanism.core.ControlBinding"), "jar dependencies too");
        assertFalse(names.stream().anyMatch(name -> name.endsWith("package-info") || name.endsWith("module-info")));
        assertTrue(names.stream().allMatch(name -> name.startsWith("ca.frc6390.athena.")));
    }

    @Test
    void loadsClassesInTheBackgroundWithoutInitializingThem() {
        ClassPreloader preloader = ClassPreloader.start("ca.frc6390.athena.wpilib.system.",
                name -> name.endsWith("ClassPreloaderTest$Probe"), ClassPreloaderTest.class.getClassLoader());
        org.junit.jupiter.api.Assertions.assertTimeoutPreemptively(Duration.ofSeconds(10), () -> {
            while (!preloader.finished()) Thread.sleep(5);
        });
        assertTrue(preloader.loadedClasses() == 1, "loaded " + preloader.loadedClasses());
        // Read from this class: touching Probe itself would initialize it.
        assertFalse(probeInitialized, "loading must not run static initializers");
    }

    static volatile boolean probeInitialized;

    static final class Probe {
        static {
            probeInitialized = true;
        }
    }
}
