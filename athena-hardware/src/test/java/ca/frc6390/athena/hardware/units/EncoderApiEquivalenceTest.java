package ca.frc6390.athena.hardware.units;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

/**
 * Pins each typed encoder declaration against the golden values of the untyped declaration it
 * replaced. The golden side is built directly from the documented formulas
 * (position = rotations * gearRatio * conversion), so this test does not depend on the removed
 * builders. Exact equality: the typed path must not change any position or velocity.
 */
class EncoderApiEquivalenceTest {
    private static final double[] ROTATIONS = {0.0, 0.25, -1.5, 3.7, 1234.5};
    private static final double[] RATES = {0.0, 1.0, -2.25, 40.0};

    private static EncoderDevice base() {
        return EncoderDevice.of(EncoderKinds.CANCODER, 2);
    }

    /** The untyped declaration as it was: gearRatio, conversion and unit set explicitly. */
    private static EncoderDevice golden(double gearRatio, double conversion, Unit units) {
        return new EncoderDevice(base().source(), false, gearRatio, conversion, 0.0,
                FailurePolicy.DISABLE_MECHANISM, false, units);
    }

    private static void assertSameNumbers(EncoderDevice expected, EncoderDevice actual) {
        assertEquals(expected.gearRatio(), actual.gearRatio(), 0.0, "gearRatio");
        assertEquals(expected.conversion(), actual.conversion(), 0.0, "conversion");
        assertEquals(expected.measuresUnit(), actual.measuresUnit(), "measuresUnit");
        for (double rotations : ROTATIONS) {
            assertEquals(expected.raw().positionFromRotations(rotations), actual.raw().positionFromRotations(rotations), 0.0);
        }
        for (double rate : RATES) {
            assertEquals(expected.raw().velocityFromRotationsPerSecond(rate),
                    actual.raw().velocityFromRotationsPerSecond(rate), 0.0);
        }
    }

    @Test
    void reductionStoresTheInverseOfTheDeclaredValue() {
        for (double reduction : new double[] {0.2, 0.5, 2.0, 4.0, 5.0, 7.0, 9.0, 10.0, 12.0, 30.0, 60.0, 100.0}) {
            assertSameNumbers(golden(1.0 / reduction, 1.0, Units.Rotations), base().reduction(reduction));
        }
    }

    @Test
    void gearingStoresTheGearRatioExactly() {
        GearRatio ratio = GearRatio.reduction(12.0, 48.0);
        assertSameNumbers(golden(ratio.ratio(), 1.0, Units.Rotations), base().gearing(ratio));
    }

    @Test
    void measuresDegreesIsOneRevolutionOf360InDegrees() {
        assertSameNumbers(golden(1.0 / 100.0, 360.0, Units.Degrees),
                base().reduction(100.0).measures(Units.Degrees));
    }

    @Test
    void measuresRotationsIsOneRevolutionPerUnit() {
        assertSameNumbers(golden(1.0, 1.0, Units.Rotations), base().measures(Units.Rotations));
    }

    @Test
    void wheelCircumferenceMatchesTheTravelPerRevolutionDeclaration() {
        for (double diameter : new double[] {0.0762, 0.1016, 0.3, 0.0508}) {
            assertSameNumbers(golden(1.0 / 6.75, Math.PI * diameter, Units.Meters),
                    base().reduction(6.75).measures(Units.Meters)
                            .travelPerRevolution(Units.Meters.of(Math.PI * diameter)));
        }
    }
}
