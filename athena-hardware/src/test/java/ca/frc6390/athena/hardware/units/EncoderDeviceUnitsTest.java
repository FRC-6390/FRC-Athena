package ca.frc6390.athena.hardware.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

class EncoderDeviceUnitsTest {

    @Test
    void measuresDegreesSetsOneRevolutionAndReportsDegrees() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Degrees);

        assertSame(Units.Degrees, encoder.measuresUnit());
        assertEquals(360.0, encoder.conversion(), 1e-9);
        assertEquals(90.0, encoder.raw().positionFromRotations(0.25), 1e-9);
    }

    @Test
    void measuresRotationsAndRadiansUseMatchingRevolutionScale() {
        assertEquals(1.0, EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Rotations).conversion(), 1e-9);
        assertEquals(2.0 * Math.PI,
                EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Radians).conversion(), 1e-9);
    }

    @Test
    void reductionAndGearingAreTypedAlternativesToGearRatio() {
        EncoderDevice byDouble = EncoderDevice.of(EncoderKinds.CANCODER, 2).reduction(10.0);
        EncoderDevice byGearing = EncoderDevice.of(EncoderKinds.CANCODER, 2)
                .gearing(GearRatio.reduction(10.0, 1.0));

        assertEquals(0.1, byDouble.gearRatio(), 0.0);
        assertEquals(0.1, byGearing.gearRatio(), 0.0);
        assertEquals(1.0, byDouble.raw().positionFromRotations(10.0), 1e-9);
    }

    @Test
    void measuresDistanceWithTravelPerRevolutionConvertsToTheDeclaredUnit() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2)
                .measures(Units.Meters)
                .travelPerRevolution(Units.Inches.of(12.0));

        assertSame(Units.Meters, encoder.measuresUnit());
        assertEquals(0.3048, encoder.conversion(), 1e-9);
    }

    @Test
    void travelPerRevolutionRequiresDistanceMeasures() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2);
        assertThrows(IllegalStateException.class, () -> encoder.travelPerRevolution(Units.Meters.of(1.0)));
        assertThrows(IllegalStateException.class,
                () -> encoder.measures(Units.Degrees).travelPerRevolution(Units.Meters.of(1.0)));
    }

    @Test
    void defaultDeclarationMeasuresRotationsOneToOne() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2);

        assertSame(Units.Rotations, encoder.measuresUnit());
        assertEquals(1.0, encoder.conversion(), 0.0);
        assertEquals(-1.5, encoder.raw().positionFromRotations(-1.5), 0.0);
    }

    @Test
    void degreesWithReductionScalesOneRawTurnTo360OverTheReduction() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Degrees).reduction(0.2);

        assertEquals(1800.0, encoder.raw().positionFromRotations(1.0), 1e-9);
    }

    @Test
    void rotationsFromPositionAcceptsAnyMeasureInTheEncoderDimension() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Degrees).reduction(100.0);

        assertEquals(25.0, encoder.rotationsFromPosition(Units.Degrees.of(90.0)), 1e-9);
        assertEquals(25.0, encoder.rotationsFromPosition(encoder.measure(90.0)), 1e-9);
    }

    @Test
    void rotationsFromPositionRejectsAMeasureOfAnotherDimension() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Degrees);

        assertThrows(IllegalArgumentException.class, () -> encoder.rotationsFromPosition(Units.Meters.of(1.0)));
    }

    @Test
    void measuredUnitSurvivesDerivedDeclarations() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2)
                .measures(Units.Degrees)
                .reduction(4.0)
                .inverted()
                .offset(0.5)
                .failurePolicy(ca.frc6390.athena.api.FailurePolicy.DISABLE_MECHANISM);

        assertSame(Units.Degrees, encoder.measuresUnit());
        assertTrue(encoder.isInverted());
        assertFalse(encoder.baselineConfigApplied());
    }

    @Test
    void canonicalConstructorRejectsMissingOrNonAngleOrDistanceUnits() {
        EncoderDevice template = EncoderDevice.of(EncoderKinds.CANCODER, 2);
        assertThrows(IllegalArgumentException.class, () -> new EncoderDevice(
                template.source(), false, 1.0, 1.0, 0.0, null, false, null));
        assertThrows(IllegalArgumentException.class, () -> new EncoderDevice(
                template.source(), false, 1.0, 1.0, 0.0, null, false, Units.Volts));
    }
}
