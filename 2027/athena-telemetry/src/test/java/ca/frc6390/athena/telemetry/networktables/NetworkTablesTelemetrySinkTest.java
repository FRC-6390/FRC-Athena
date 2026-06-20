package ca.frc6390.athena.telemetry.networktables;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.telemetry.TelemetryKey;
import ca.frc6390.athena.telemetry.TelemetryRegistry;

class NetworkTablesTelemetrySinkTest {
    @Test
    void normalizesTelemetryPathsUnderAthenaRoot() {
        assertEquals("/Athena/robot/enabled", NetworkTablePath.from(TelemetryKey.bool("/robot//enabled")));
        assertEquals("/Robot/drive/velocity", NetworkTablePath.from("/Robot/", TelemetryKey.number("drive/velocity")));
    }

    @Test
    void publishesTypedValuesToWriter() {
        InMemoryNetworkTableWriter writer = new InMemoryNetworkTableWriter();
        NetworkTablesTelemetrySink sink = new NetworkTablesTelemetrySink(writer);

        new TelemetryRegistry()
                .booleanValue(TelemetryKey.bool("robot/enabled"), () -> true)
                .numberValue(TelemetryKey.number("battery/voltage"), () -> 12.3)
                .stringValue(TelemetryKey.string("auto/selected"), () -> "leave")
                .publishAll(sink);

        assertEquals(true, writer.values().get("/Athena/robot/enabled"));
        assertEquals(12.3, writer.values().get("/Athena/battery/voltage"));
        assertEquals("leave", writer.values().get("/Athena/auto/selected"));
    }

    @Test
    void customRootIsNormalized() {
        InMemoryNetworkTableWriter writer = new InMemoryNetworkTableWriter();
        NetworkTablesTelemetrySink sink = new NetworkTablesTelemetrySink("///AthenaTest//", writer);

        sink.publish(TelemetryKey.string("/mode"), ca.frc6390.athena.telemetry.TelemetryValue.of("sim"));

        assertEquals("sim", writer.values().get("/AthenaTest/mode"));
    }
}
