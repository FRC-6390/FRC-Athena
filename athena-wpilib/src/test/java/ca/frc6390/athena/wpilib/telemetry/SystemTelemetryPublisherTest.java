package ca.frc6390.athena.wpilib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.wpilib.system.MemoryPressure;
import ca.frc6390.athena.wpilib.system.SystemStatus;
import ca.frc6390.athena.wpilib.system.SystemTuning;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import org.junit.jupiter.api.Test;

class SystemTelemetryPublisherTest {
    @Test
    void publishesMemoryPressureAndTuningResults() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        SystemStatus status = new SystemStatus(
                SystemTuning.Profile.AUTOMATIC,
                "roboRIO 1.0",
                MemoryPressure.WARNING,
                1000, 100, 250, 150, 500, 20, 32, 4,
                true,
                List.of("swap enabled"),
                List.of("web server permission denied"));
        try (SystemTelemetryPublisher publisher = new SystemTelemetryPublisher(instance)) {
            publisher.publish(status);

            assertEquals("AUTOMATIC", instance.getEntry(SystemTelemetryPublisher.ROOT + "/Profile").getString(""));
            assertEquals("WARNING", instance.getEntry(SystemTelemetryPublisher.ROOT + "/Pressure").getString(""));
            assertEquals(100, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Memory/AvailableBytes").getInteger(-1));
            assertEquals(250, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Memory/ProcessResidentBytes").getInteger(-1));
            assertEquals(20, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Memory/DirectBufferBytes").getInteger(-1));
            assertTrue(instance.getEntry(SystemTelemetryPublisher.ROOT + "/Tuning/Complete").getBoolean(false));
            assertArrayEquals(new String[] {"swap enabled"}, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Tuning/AppliedChanges").getStringArray(new String[0]));
            assertArrayEquals(new String[] {"web server permission denied"}, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Tuning/Failures").getStringArray(new String[0]));

            long lastChange = instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Memory/AvailableBytes").getLastChange();
            publisher.publish(status);
            assertEquals(lastChange, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Memory/AvailableBytes").getLastChange());
        } finally {
            instance.close();
        }
    }
}
