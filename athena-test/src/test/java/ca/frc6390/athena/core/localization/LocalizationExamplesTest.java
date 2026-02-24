package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.LocalizationExamples;
import org.junit.jupiter.api.Test;

final class LocalizationExamplesTest {

    @Test
    void competitionConfigExampleAppliesBackendAndZones() {
        RobotLocalizationConfig cfg = LocalizationExamples.createCompetitionConfig();

        assertTrue(cfg.useVision());
        assertEquals("loading", cfg.autoPoseName());
        assertEquals(2, cfg.boundingBoxes().size());

        RobotLocalizationConfig.BackendConfig backend = cfg.backend();
        assertEquals(0.45, backend.slipYawRateThreshold(), 1e-9);
        assertEquals(1.2, backend.slipAccelThreshold(), 1e-9);
        assertEquals(0.8, backend.poseJumpMeters(), 1e-9);
    }
}
