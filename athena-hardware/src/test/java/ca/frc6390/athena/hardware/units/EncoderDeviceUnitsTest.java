package ca.frc6390.athena.hardware.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

class EncoderDeviceUnitsTest {

    @Test
    void measuresDegreesSetsOneRevolutionAndReportsDegrees() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Degrees);

        assertSame(Units.Degrees, encoder.measuresUnit());
        assertEquals(EncoderUnit.DEGREES, encoder.units());
        assertEquals(360.0, encoder.conversion(), 1e-9);
        assertEquals(90.0, encoder.positionFromRotations(0.25), 1e-9);
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
        assertEquals(1.0, byDouble.positionFromRotations(10.0), 1e-9);
    }

    @Test
    void measuresDistanceWithTravelPerRevolutionConvertsToTheDeclaredUnit() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2)
                .measures(Units.Meters)
                .travelPerRevolution(Units.Inches.of(12.0));

        assertSame(Units.Meters, encoder.measuresUnit());
        assertEquals(EncoderUnit.METERS, encoder.units());
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
    void legacyDeclarationsHaveNoMeasuredUnitAndKeepOldConversions() {
        EncoderDevice legacy = EncoderDevice.of(EncoderKinds.CANCODER, 2).reduction(0.2).conversion(360.0);

        assertNull(legacy.measuresUnit());
        assertEquals(EncoderUnit.RAW, legacy.units());
        assertEquals(1800.0, legacy.positionFromRotations(1.0), 1e-9);
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
    void primitiveConstructorRejectsNonAngleOrDistanceUnits() {
        assertThrows(IllegalArgumentException.class, () -> new EncoderDevice(
                new ca.frc6390.athena.hardware.device.EncoderDevice.EncoderSource.Standalone(
                        EncoderKinds.CANCODER, "rio", new ca.frc6390.athena.hardware.device.HardwareAddress.Can(2)),
                false, 1.0, 1.0, 0.0, EncoderUnit.RAW, null, false, Units.Volts));
    }
}
