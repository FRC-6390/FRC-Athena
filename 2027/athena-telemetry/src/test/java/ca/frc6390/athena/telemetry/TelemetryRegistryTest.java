package ca.frc6390.athena.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.LinkedHashMap;
import java.util.Map;

import org.junit.jupiter.api.Test;

class TelemetryRegistryTest {
    @Test
    void capturesSnapshot() {
        TelemetryKey enabled = TelemetryKey.bool("robot/enabled");
        TelemetryKey voltage = TelemetryKey.number("robot/voltage");

        TelemetrySnapshot snapshot = new TelemetryRegistry()
                .booleanValue(enabled, () -> true)
                .numberValue(voltage, () -> 12.4)
                .snapshot();

        assertEquals(TelemetryValue.of(true), snapshot.find(enabled).orElseThrow());
        assertEquals(TelemetryValue.of(12.4), snapshot.find(voltage).orElseThrow());
    }

    @Test
    void publishesInRegistrationOrder() {
        TelemetryKey state = TelemetryKey.string("intake/state");
        Map<String, TelemetryValue> published = new LinkedHashMap<>();

        new TelemetryRegistry()
                .stringValue(state, () -> "running")
                .publishAll((key, value) -> published.put(key.path(), value));

        assertEquals(TelemetryValue.of("running"), published.get("intake/state"));
    }

    @Test
    void rejectsMismatchedTypeRegistration() {
        TelemetryRegistry registry = new TelemetryRegistry();
        TelemetryKey key = TelemetryKey.number("bad");

        assertThrows(IllegalArgumentException.class, () -> registry.booleanValue(key, () -> true));
    }
}
