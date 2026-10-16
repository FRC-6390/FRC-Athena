package ca.frc6390.athena.vision.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class VisionSimNoiseTest {
    @Test
    void realisticDefaultsAreTheDocumentedModel() {
        VisionSimNoise realistic = VisionSimNoise.realistic();

        assertEquals(30.0, realistic.fps(), 0.0);
        assertEquals(25.0, realistic.latencyMs(), 0.0);
        assertEquals(5.0, realistic.jitterMs(), 0.0);
        assertEquals(0.005, realistic.translationStdAt1m(), 0.0);
        assertEquals(0.1, realistic.headingStdDegAt1m(), 0.0);
        assertEquals(4.0, realistic.singleTagMultiplier(), 0.0);
        assertEquals(0.02, realistic.dropProbability(), 0.0);
        assertEquals(0.05, realistic.ambiguityFlipProbability(), 0.0);
        assertEquals(8.0, realistic.maxRangeMeters(), 0.0);
        assertFalse(realistic.isPerfect());
    }

    @Test
    void perfectModelIsExactAndUnlimited() {
        VisionSimNoise perfect = VisionSimNoise.perfect();

        assertTrue(perfect.isPerfect());
        assertTrue(Double.isInfinite(perfect.fps()));
        assertTrue(Double.isInfinite(perfect.maxRangeMeters()));
        assertEquals(0.0, perfect.latencyMs(), 0.0);
        assertEquals(0.0, perfect.dropProbability(), 0.0);
    }

    @Test
    void knobsReturnCopiesWithOneValueChanged() {
        VisionSimNoise base = VisionSimNoise.realistic();
        VisionSimNoise changed = base.fps(10.0).latencyMs(40.0).jitterMs(2.0).dropProbability(0.1)
                .ambiguityFlipProbability(0.3).seed(7L);

        assertEquals(10.0, changed.fps(), 0.0);
        assertEquals(40.0, changed.latencyMs(), 0.0);
        assertEquals(2.0, changed.jitterMs(), 0.0);
        assertEquals(0.1, changed.dropProbability(), 0.0);
        assertEquals(0.3, changed.ambiguityFlipProbability(), 0.0);
        assertEquals(7L, changed.seed());
        assertEquals(base.translationStdAt1m(), changed.translationStdAt1m(), 0.0);
        assertEquals(30.0, base.fps(), 0.0);
    }

    @Test
    void invalidKnobsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> VisionSimNoise.realistic().fps(0.0));
        assertThrows(IllegalArgumentException.class, () -> VisionSimNoise.realistic().fps(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> VisionSimNoise.realistic().latencyMs(-1.0));
        assertThrows(IllegalArgumentException.class, () -> VisionSimNoise.realistic().dropProbability(1.5));
        assertThrows(IllegalArgumentException.class,
                () -> VisionSimNoise.realistic().ambiguityFlipProbability(-0.1));
        assertThrows(IllegalArgumentException.class, () -> VisionSimNoise.realistic().maxRangeMeters(0.0));
    }

    @Test
    void fieldDefaultsToRealisticNoiseAndKeepsItsNoiseWhenTargetsAreAdded() {
        VisionSimulationField field = VisionSimulationField.of(
                VisionSimulationTarget.aprilTag(1, 2.0, 0.0, 1.0, 0.0));

        assertEquals(VisionSimNoise.realistic(), field.noise());
        assertEquals(VisionSimNoise.perfect(), field.noise(VisionSimNoise.perfect()).noise());
        assertEquals(VisionSimNoise.perfect(),
                field.noise(VisionSimNoise.perfect()).withTarget(VisionSimulationTarget.aprilTag(2, 3.0, 0.0, 1.0, 0.0))
                        .noise());
        assertEquals(VisionSimNoise.realistic(), field.noise(null).noise());
    }
}
