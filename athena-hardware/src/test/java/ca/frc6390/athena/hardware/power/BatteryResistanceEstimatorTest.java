package ca.frc6390.athena.hardware.power;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class BatteryResistanceEstimatorTest {
    @Test
    void recoversResistanceFromVaryingLoad() {
        double voc = 12.8;
        double r = 0.030;
        var estimator = new BatteryResistanceEstimator(20.0, 0.02, 0.02);
        for (int i = 0; i < 2000; i++) {
            // Load swings between 0 and 120 A so current variance is large.
            double current = 60.0 + 60.0 * Math.sin(i * 0.3);
            estimator.sample(voc - r * current, current);
        }
        assertTrue(estimator.excited());
        assertEquals(r, estimator.ohms(), 0.002);
        assertEquals(voc, estimator.openCircuitVolts(), 0.05);
    }

    @Test
    void steadyLoadKeepsInitialResistance() {
        var estimator = new BatteryResistanceEstimator(20.0, 0.02, 0.02);
        for (int i = 0; i < 500; i++) {
            estimator.sample(12.0 - 0.1 * 50.0, 50.0);
        }
        assertFalse(estimator.excited());
        assertEquals(0.02, estimator.ohms(), 1e-12);
    }

    @Test
    void ignoresNonFiniteSamples() {
        var estimator = new BatteryResistanceEstimator(20.0, 0.02, 0.02);
        estimator.sample(Double.NaN, 10.0);
        estimator.sample(12.0, Double.POSITIVE_INFINITY);
        assertEquals(0, estimator.samples());
        assertTrue(Double.isNaN(estimator.openCircuitVolts()));
    }
}
