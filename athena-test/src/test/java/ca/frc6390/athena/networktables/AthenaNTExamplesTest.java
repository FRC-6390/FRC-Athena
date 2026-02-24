package ca.frc6390.athena.networktables;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.networktables.examples.AthenaNTExamples;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class AthenaNTExamplesTest {
    private NetworkTableInstance instance;

    @BeforeEach
    void setUp() {
        instance = NetworkTableInstance.create();
        instance.startLocal();
        AthenaNT.useInstanceForTests(instance);
        AthenaNT.clearBindingsForTests();
    }

    @AfterEach
    void tearDown() {
        AthenaNT.clearBindingsForTests();
        AthenaNT.useInstanceForTests(NetworkTableInstance.getDefault());
        if (instance != null) {
            instance.close();
            instance = null;
        }
    }

    @Test
    void publishAndReadTelemetryRoundTripsValues() {
        AthenaNTExamples.publishDrivetrainTelemetry(true, 91.0, "AUTO");

        assertEquals(91.0, AthenaNTExamples.readHeadingDegrees(0.0), 1e-9);
        assertEquals("AUTO", AthenaNT.getString("Drive/Mode", "NONE"));
    }

    @Test
    void scopedBindingWritesMetrics() {
        SampleBindings sample = new SampleBindings();
        AthenaNTBinding binding = AthenaNTExamples.bind("Subsystem/Arm", sample);

        try {
            AthenaNTExamples.tickBindings();
            assertEquals(42.0, AthenaNT.getDouble("Subsystem/Arm/Telemetry/positionDeg", 0.0), 1e-9);
        } finally {
            binding.close();
        }
    }

    private static final class SampleBindings {
        @AthenaNTMetric(key = "positionDeg", scope = "Telemetry", periodMs = 1)
        private double positionDeg = 42.0;
    }
}
