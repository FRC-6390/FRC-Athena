package ca.frc6390.athena.wpilib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LoopTimingPublisherTest {
    private NetworkTableInstance instance;
    private long now;

    @BeforeEach
    void create() {
        instance = NetworkTableInstance.create();
    }

    @AfterEach
    void close() {
        instance.close();
    }

    @Test
    void publishesWindowAveragesMaximaAndOverruns() {
        try (LoopTimingPublisher publisher = new LoopTimingPublisher(instance, () -> now)) {
            // 49 cycles of 4 ms work every 20 ms, then one 30 ms overrun period with 12 ms of work.
            for (int cycle = 0; cycle < 50; cycle++) {
                publisher.cycleStarted();
                now += cycle == 49 ? 12_000_000L : 4_000_000L;
                publisher.cycleFinished();
                now += cycle == 48 ? 26_000_000L : 16_000_000L;
            }
            // The 50th cycle ends 1002 ms after the window opened, so the window is published then.
            double work = entry("WorkMillis");
            assertEquals((49 * 4.0 + 12.0) / 50.0, work, 1e-9);
            assertEquals(12.0, entry("MaxWorkMillis"), 1e-9);
            assertEquals(30.0, entry("MaxPeriodMillis"), 1e-9);
            assertEquals((48 * 20.0 + 30.0) / 49.0, entry("PeriodMillis"), 1e-9);
            assertEquals(1, instance.getEntry(LoopTimingPublisher.ROOT + "/Overruns").getInteger(-1));
            assertEquals(work / entry("PeriodMillis"), entry("Load"), 1e-9);
        }
    }

    private double entry(String name) {
        return instance.getEntry(LoopTimingPublisher.ROOT + "/" + name).getDouble(Double.NaN);
    }
}
