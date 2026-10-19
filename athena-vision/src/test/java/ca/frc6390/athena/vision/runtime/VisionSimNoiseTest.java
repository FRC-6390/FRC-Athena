package ca.frc6390.athena.vision.runtime;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class VisionSimNoiseTest {
    @Test
    void realisticDefaultsAreTheDocumentedModel() {
        VisionSimNoise realistic = VisionSimNoise.realistic();

        assertEquals(30.0, realistic.fps().in(Hertz), 0.0);
        assertEquals(25.0, realistic.latency().in(Milliseconds), 0.0);
        assertEquals(0.025, realistic.latency().in(Seconds), 1.0e-12);
        assertEquals(5.0, realistic.jitter().in(Milliseconds), 0.0);
        assertEquals(0.005, realistic.translationStdAt1m().in(Meters), 0.0);
        assertEquals(0.1, realistic.headingStdAt1m().in(Degrees), 0.0);
        assertEquals(4.0, realistic.singleTagMultiplier(), 0.0);
        assertEquals(0.02, realistic.dropProbability(), 0.0);
        assertEquals(0.05, realistic.ambiguityFlipProbability(), 0.0);
        assertEquals(8.0, realistic.maxRange().in(Meters), 0.0);
        assertFalse(realistic.isPerfect());
    }

    @Test
    void headingStdIsDegreesAndMatchesTheRadianValue() {
        VisionSimNoise realistic = VisionSimNoise.realistic();

        assertEquals(Math.toRadians(0.1), realistic.headingStdAt1m().in(Radians), 1.0e-12);
    }

    @Test
    void perfectModelIsExactAndUnlimited() {
        VisionSimNoise perfect = VisionSimNoise.perfect();

        assertTrue(perfect.isPerfect());
        assertTrue(Double.isInfinite(perfect.fps().in(Hertz)));
        assertTrue(Double.isInfinite(perfect.maxRange().in(Meters)));
        assertEquals(0.0, perfect.latency().in(Milliseconds), 0.0);
        assertEquals(0.0, perfect.dropProbability(), 0.0);
    }

    @Test
    void knobsReturnCopiesWithOneValueChanged() {
        VisionSimNoise base = VisionSimNoise.realistic();
        VisionSimNoise changed = base.fps(Hertz.of(10.0)).latency(Milliseconds.of(40.0))
                .jitter(Milliseconds.of(2.0)).dropProbability(0.1)
                .ambiguityFlipProbability(0.3).seed(7L);

        assertEquals(10.0, changed.fps().in(Hertz), 0.0);
        assertEquals(40.0, changed.latency().in(Milliseconds), 0.0);
        assertEquals(2.0, changed.jitter().in(Milliseconds), 0.0);
        assertEquals(0.1, changed.dropProbability(), 0.0);
        assertEquals(0.3, changed.ambiguityFlipProbability(), 0.0);
        assertEquals(7L, changed.seed());
        assertEquals(base.translationStdAt1m(), changed.translationStdAt1m());
        assertEquals(30.0, base.fps().in(Hertz), 0.0);
    }

    @Test
    void invalidKnobsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> VisionSimNoise.realistic().fps(Hertz.of(0.0)));
        assertThrows(IllegalArgumentException.class, () -> VisionSimNoise.realistic().fps(Hertz.of(Double.NaN)));
        assertThrows(IllegalArgumentException.class,
                () -> VisionSimNoise.realistic().latency(Milliseconds.of(-1.0)));
        assertThrows(IllegalArgumentException.class, () -> VisionSimNoise.realistic().dropProbability(1.5));
        assertThrows(IllegalArgumentException.class,
                () -> VisionSimNoise.realistic().ambiguityFlipProbability(-0.1));
        assertThrows(IllegalArgumentException.class, () -> VisionSimNoise.realistic().maxRange(Meters.of(0.0)));
    }

    @Test
    void fieldDefaultsToRealisticNoiseAndKeepsItsNoiseWhenTargetsAreAdded() {
        VisionSimulationField field = VisionSimulationField.of(
                VisionSimulationTarget.aprilTag(1, Meters.of(2.0), Meters.of(0.0), Meters.of(1.0), Radians.of(0.0)));

        assertEquals(VisionSimNoise.realistic().fps(), field.noise().fps());
        assertTrue(field.noise(VisionSimNoise.perfect()).noise().isPerfect());
        assertTrue(field.noise(VisionSimNoise.perfect())
                .withTarget(VisionSimulationTarget.aprilTag(2, Meters.of(3.0), Meters.of(0.0),
                        Meters.of(1.0), Radians.of(0.0)))
                .noise().isPerfect());
        assertEquals(VisionSimNoise.realistic().jitter(), field.noise(null).noise().jitter());
    }
}
