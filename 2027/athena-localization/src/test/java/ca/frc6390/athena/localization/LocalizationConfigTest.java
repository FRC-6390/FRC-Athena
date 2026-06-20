package ca.frc6390.athena.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.localization.config.LocalizationConfig;
import ca.frc6390.athena.localization.config.Localizations;

class LocalizationConfigTest {
    @Test
    void lowersLocalizationConfig() {
        var spec = Localizations.localization("robotPose", localization -> localization
                .vision(vision -> vision
                        .standardDeviations(0.8, 0.85, 0.6)
                        .multiTagScale(0.45))
                .slip(slip -> slip.enabled(0.2, 0.4))
                .fieldBounds("chargedUp", 0.0, 0.0, 16.54, 8.02)
                .poseAlias("speaker", 1.2, 5.5, 0.0));

        assertEquals("robotPose", spec.name());
        assertEquals(0.8, spec.visionWeight().xStdDevMeters(), 1.0e-9);
        assertEquals(0.36, spec.visionWeight().forTagCount(2).xStdDevMeters(), 1.0e-9);
        assertEquals(0.2, spec.slipDetection().lateralVelocityMetersPerSecond(), 1.0e-9);
        assertEquals("chargedUp", spec.fieldBounds().name());
        assertTrue(spec.findFieldBounds("chargedUp").isPresent());
        assertEquals(1.2, spec.findPoseAlias("speaker").orElseThrow().pose().xMeters(), 1.0e-9);
        assertFalse(spec.validate().hasErrors());
    }

    @Test
    void poseAliasReplacementKeepsLastValue() {
        var spec = LocalizationConfig.create("robotPose")
                .poseAlias("start", 1.0, 1.0, 0.0)
                .poseAlias("start", 2.0, 3.0, 1.57)
                .toSpec();

        assertEquals(1, spec.poseAliases().size());
        assertEquals(2.0, spec.findPoseAlias("start").orElseThrow().pose().xMeters(), 1.0e-9);
    }

    @Test
    void slipDetectionCanBeDisabled() {
        var spec = LocalizationConfig.create("robotPose")
                .slip(slip -> slip.disabled())
                .toSpec();

        assertFalse(spec.slipDetection().enabled());
        assertFalse(spec.validate().hasErrors());
    }

    @Test
    void invalidValuesReportErrors() {
        var spec = LocalizationConfig.create("robotPose")
                .vision(vision -> vision.standardDeviations(Double.NaN, 0.8, 0.6))
                .slip(slip -> slip.enabled(-0.1, 0.4))
                .fieldBounds("field", 5.0, 0.0, 1.0, 8.0)
                .poseAlias("bad", Double.POSITIVE_INFINITY, 0.0, 0.0)
                .toSpec();

        var report = spec.validate();

        assertTrue(report.hasErrors());
        assertFalse(report.errorsWithCode("localization.invalid-vision-weight").isEmpty());
        assertFalse(report.errorsWithCode("localization.invalid-slip-detection").isEmpty());
        assertFalse(report.errorsWithCode("localization.invalid-field-bounds").isEmpty());
        assertFalse(report.errorsWithCode("localization.invalid-pose-alias").isEmpty());
    }
}
