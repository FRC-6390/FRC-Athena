package ca.frc6390.athena.networktables;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AthenaNTTest {
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
    void topLevelPutAndTypedGet() {
        AthenaNT.put("Drive/Enabled", true);
        AthenaNT.put("Drive/Gear", 2);
        AthenaNT.put("Drive/HeadingDeg", 137.5);
        AthenaNT.put("Drive/Mode", "AUTO");
        AthenaNT.put("Drive/Wheels", new double[] {1.0, 2.0, 3.0, 4.0});

        assertTrue(AthenaNT.getBoolean("Drive/Enabled", false));
        assertEquals(2L, AthenaNT.getInt("Drive/Gear", 0));
        assertEquals(137.5, AthenaNT.getDouble("Drive/HeadingDeg", 0.0), 1e-9);
        assertEquals("AUTO", AthenaNT.getString("Drive/Mode", "NONE"));

        double[] wheels = AthenaNT.get("Drive/Wheels", double[].class, new double[0]);
        assertEquals(4, wheels.length);
        assertEquals(3.0, wheels[2], 1e-9);
    }

    @Test
    void complexObjectRoundTripUsesJson() {
        ShotPlan expected = new ShotPlan(4200.0, 28.0, true);
        AthenaNT.put("Auto/ShotPlan", expected);

        ShotPlan decoded = AthenaNT.get("Auto/ShotPlan", ShotPlan.class, new ShotPlan(0.0, 0.0, false));
        assertEquals(expected, decoded);
    }

    @Test
    void scopedAccessAndAnnotationsWork() {
        SampleBindings sample = new SampleBindings();
        NtScope scope = AthenaNT.scope("Mechanisms/Arm/NetworkTables");
        AthenaNTBinding binding = scope.bind(sample);

        try {
            AthenaNT.tick();
            assertEquals(
                    12.0,
                    AthenaNT.getDouble("Mechanisms/Arm/NetworkTables/Arm/Telemetry/positionDeg", 0.0),
                    1e-9);

            AthenaNT.put("Mechanisms/Arm/NetworkTables/Arm/Tuning/kP", 0.55);
            AthenaNT.tick();
            assertEquals(0.55, sample.kP, 1e-9);
            assertEquals(1, sample.kPChangeCount);

            AthenaNT.put("Mechanisms/Arm/NetworkTables/Arm/Actions/ZeroEncoder", true);
            AthenaNT.tick();
            AthenaNT.tick();
            assertEquals(1, sample.zeroCount);
        } finally {
            binding.close();
        }
    }

    private record ShotPlan(double rpm, double hoodDeg, boolean movingAllowed) {}

    @AthenaNTScope("Arm")
    private static final class SampleBindings {
        @AthenaNTMetric(key = "positionDeg", scope = "Telemetry", periodMs = 1)
        private double positionDeg = 12.0;

        @AthenaNTTunable(key = "kP")
        private double kP = 0.25;

        private int kPChangeCount;
        private int zeroCount;

        @AthenaNTOnChange(key = "kP")
        private void onKpChange(double value) {
            kP = value;
            kPChangeCount++;
        }

        @AthenaNTAction(key = "ZeroEncoder")
        private void zeroEncoder() {
            zeroCount++;
        }
    }
}
