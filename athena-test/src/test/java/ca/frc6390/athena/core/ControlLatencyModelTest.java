package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

final class ControlLatencyModelTest {

    @Test
    void sourceAgeUsesPositiveDeltaOnly() {
        assertEquals(0.15, ControlLatencyModel.sourceAgeSeconds(10.0, 9.85), 1e-9);
        assertEquals(0.0, ControlLatencyModel.sourceAgeSeconds(10.0, 10.5), 1e-9);
        assertEquals(0.0, ControlLatencyModel.sourceAgeSeconds(Double.NaN, 1.0), 1e-9);
    }

    @Test
    void effectiveInputLatencyIncludesSampleAndHoldAndPipeline() {
        double latency = ControlLatencyModel.effectiveInputLatencySeconds(
                20.0,
                19.98,
                0.02,
                0.005,
                1.0);
        // age (0.02) + half period (0.01) + pipeline (0.005)
        assertEquals(0.035, latency, 1e-9);
    }

    @Test
    void effectiveInputLatencyClampsToConfiguredMaximum() {
        double latency = ControlLatencyModel.effectiveInputLatencySeconds(
                50.0,
                49.0,
                0.1,
                0.2,
                0.25);
        assertEquals(0.25, latency, 1e-9);
    }

    @Test
    void predictiveLeadAddsAdaptiveAndClamps() {
        assertEquals(0.06, ControlLatencyModel.predictiveLeadSeconds(0.05, 0.01, 0.2), 1e-9);
        assertEquals(0.08, ControlLatencyModel.predictiveLeadSeconds(0.05, 0.05, 0.08), 1e-9);
        assertEquals(0.0, ControlLatencyModel.predictiveLeadSeconds(Double.NaN, Double.NaN, 0.08), 1e-9);
    }
}
