package ca.frc6390.athena.hardware.runtime;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Hardware declarations create device actions without a compile-time or reflective dependency on
 * athena-mechanisms. The class files are checked as bytes, so the check needs no source tree.
 */
class DeviceActionLayeringTest {
    private static final List<String> HARDWARE_CLASSES = List.of(
            "runtime/DeviceAction",
            "runtime/DeviceActionFactories",
            "runtime/DeviceActionFactory",
            "device/MotorDevice",
            "device/EncoderDevice",
            "signal/Imus",
            "signal/ImuSource",
            "power/PowerActions",
            "power/PowerProfile");

    @Test
    void hardwareActionFactoriesHaveNoMechanismReferenceOrReflectiveLookup() throws IOException {
        for (String name : HARDWARE_CLASSES) {
            String bytes = classBytes(name);
            assertFalse(bytes.contains("ca/frc6390/athena/mechanism"), name + " references athena-mechanisms");
            assertFalse(bytes.contains("forName"), name + " uses Class.forName");
            assertFalse(bytes.contains("getDeclaredMethod"), name + " looks up methods reflectively");
        }
    }

    @Test
    void deviceActionsFailClearlyWithoutAthenaMechanisms() {
        IllegalStateException failure = assertThrows(IllegalStateException.class, DeviceActionFactories::get);

        assertTrue(failure.getMessage().contains("athena-mechanisms"), failure.getMessage());
    }

    private static String classBytes(String name) throws IOException {
        String resource = "ca/frc6390/athena/hardware/" + name + ".class";
        try (InputStream in = DeviceAction.class.getClassLoader().getResourceAsStream(resource)) {
            assertTrue(in != null, "missing class resource " + resource);
            return new String(in.readAllBytes(), StandardCharsets.ISO_8859_1);
        }
    }
}
