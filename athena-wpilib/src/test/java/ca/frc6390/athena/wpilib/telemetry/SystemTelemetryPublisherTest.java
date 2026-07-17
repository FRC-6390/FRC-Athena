package ca.frc6390.athena.wpilib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.wpilib.system.MemoryPressure;
import ca.frc6390.athena.wpilib.system.SystemStatus;
import ca.frc6390.athena.wpilib.system.SystemTuning;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import org.junit.jupiter.api.Test;

class SystemTelemetryPublisherTest {
    @Test
    void publishesHealthTrendsRecommendationsAndTuningResults() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        SystemStatus status = new SystemStatus(
                SystemTuning.Profile.AUTOMATIC, "roboRIO 1.0", MemoryPressure.WARNING, "RAM is low",
                1000, 100, 90, 250, 150, 200, 500, 30, 20,
                2.0, 50, 3.0, 4.0, 123.0, -10.0, 10.0, 4, 50, 0.1, 0.2, 0.3, 12,
                32, 4, "ZRAM", 2, true, false, false,
                List.of("-Xms32m", "-Xmx128m"),
                List.of("swap enabled"), List.of("web server permission denied"));
        try (SystemTelemetryPublisher publisher = new SystemTelemetryPublisher(instance)) {
            publisher.publish(status);

            assertEquals("WARNING", instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Pressure/Level").getString(""));
            assertEquals("RAM is low", instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Pressure/Reason").getString(""));
            assertEquals(100, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Memory/AvailableBytes").getInteger(-1));
            assertEquals(20, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/JVM/DirectBufferBytes").getInteger(-1));
            assertEquals(123.0, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/JVM/AllocationRateBytesPerSecond").getDouble(-1));
            assertEquals("ZRAM", instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/Swap/Kind").getString(""));
            assertTrue(instance.getEntry(SystemTelemetryPublisher.ROOT + "/Tuning/Complete").getBoolean(false));
            assertFalse(instance.getEntry(SystemTelemetryPublisher.ROOT + "/Tuning/Verified").getBoolean(true));
            assertArrayEquals(new String[] {"-Xms32m", "-Xmx128m"}, instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/JVM/RecommendedArguments").getStringArray(new String[0]));
            assertEquals("jvmArgs.addAll(\"-Xms32m\", \"-Xmx128m\")", instance.getEntry(
                    SystemTelemetryPublisher.ROOT + "/JVM/GradleRioSnippet").getString(""));

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
