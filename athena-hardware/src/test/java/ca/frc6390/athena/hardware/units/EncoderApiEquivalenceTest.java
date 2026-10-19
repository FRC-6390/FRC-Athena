package ca.frc6390.athena.hardware.units;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

/**
 * Pins that each typed encoder declaration produces the same numbers as the untyped declaration it
 * replaces. Exact equality: the typed path must not change any position or velocity.
 */
class EncoderApiEquivalenceTest {
    private static final double[] ROTATIONS = {0.0, 0.25, -1.5, 3.7, 1234.5};
    private static final double[] RATES = {0.0, 1.0, -2.25, 40.0};

    private static EncoderDevice base() {
        return EncoderDevice.of(EncoderKinds.CANCODER, 2);
    }

    private static void assertSameNumbers(EncoderDevice expected, EncoderDevice actual) {
        assertEquals(expected.gearRatio(), actual.gearRatio(), 0.0, "gearRatio");
        assertEquals(expected.conversion(), actual.conversion(), 0.0, "conversion");
        assertEquals(expected.units(), actual.units(), "units");
        for (double rotations : ROTATIONS) {
            assertEquals(expected.positionFromRotations(rotations), actual.positionFromRotations(rotations), 0.0);
            assertEquals(expected.rotationsFromPosition(rotations), actual.rotationsFromPosition(rotations), 0.0);
        }
        for (double rate : RATES) {
            assertEquals(expected.velocityFromRotationsPerSecond(rate),
                    actual.velocityFromRotationsPerSecond(rate), 0.0);
        }
    }

    @Test
    void reductionMatchesInverseGearRatioForEveryDeclaredReduction() {
        for (double reduction : new double[] {0.2, 0.5, 2.0, 4.0, 5.0, 7.0, 9.0, 10.0, 12.0, 30.0, 60.0, 100.0}) {
            assertSameNumbers(base().gearRatio(1.0 / reduction), base().reduction(reduction));
        }
    }

    @Test
    void gearingMatchesGearRatioObjectExactly() {
        GearRatio ratio = GearRatio.reduction(12.0, 48.0);
        assertSameNumbers(base().gearRatio(ratio), base().gearing(ratio));
    }

    @Test
    void measuresDegreesMatchesDegreesConversionAndUnit() {
        assertSameNumbers(
                base().gearRatio(GearRatio.reduction(100.0, 1.0)).conversion(360.0).units(EncoderUnit.DEGREES),
                base().reduction(100.0).measures(Units.Degrees));
    }

    @Test
    void measuresRotationsMatchesRotationsUnit() {
        assertSameNumbers(
                base().units(EncoderUnit.ROTATIONS),
                base().measures(Units.Rotations));
    }

    @Test
    void wheelDiameterMatchesTravelPerRevolution() {
        for (double diameter : new double[] {0.0762, 0.1016, 0.3, 0.0508}) {
            assertSameNumbers(
                    base().gearRatio(GearRatio.reduction(6.75, 1.0)).wheelDiameterMeters(diameter)
                            .units(EncoderUnit.METERS),
                    base().reduction(6.75).measures(Units.Meters)
                            .travelPerRevolution(Units.Meters.of(Math.PI * diameter)));
        }
    }
}
