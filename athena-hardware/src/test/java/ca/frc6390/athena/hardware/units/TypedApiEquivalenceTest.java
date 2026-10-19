package ca.frc6390.athena.hardware.units;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.power.PowerProfile;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

/** Checks that typed declarations produce the same numbers the primitive forms produced. */
class TypedApiEquivalenceTest {
    @Test
    void degreeRangeKeepsDeclaredNumbersAndClampsInDegrees() {
        Range range = Range.of(Units.Degrees.of(-270.0), Units.Degrees.of(270.0));

        assertEquals(Units.Degrees, range.unit());
        assertEquals(-270.0, range.minimumInUnit(), 0.0);
        assertEquals(270.0, range.maximumInUnit(), 0.0);
        assertEquals(270.0, range.clamp(300.0), 0.0);
        assertEquals(-45.0, range.clamp(-45.0), 0.0);
    }

    @Test
    void degreeAndRotationBoundsAgreeAfterConversion() {
        // 90 degrees is one quarter turn; the typed conversion must match the old degrees/360 division.
        assertEquals(0.25, Units.Degrees.of(90.0).in(Units.Rotations), 1e-12);
        assertEquals(90.0 / 360.0, Units.Degrees.of(90.0).in(Units.Rotations), 1e-12);
    }

    @Test
    void rpmDeclaredMotorMatchesTheRadiansPerSecondFormula() {
        MotorConstants motor = MotorConstants.of(
                Units.NewtonMeters.of(2.0),
                Units.Amps.of(100.0),
                Units.Amps.of(2.0),
                Units.RPM.of(6000.0));
        double freeSpeed = 6000.0 * 2.0 * Math.PI / 60.0;

        assertEquals(freeSpeed, motor.freeSpeed().in(Units.RadiansPerSecond), 1e-9);
        assertEquals(0.12, motor.resistance().in(Units.Ohms), 1e-12);
        assertEquals(freeSpeed / (12.0 - 0.12 * 2.0), motor.speedConstantRadPerSecPerVolt(), 1e-9);
        assertEquals(0.02 * 3.0, motor.torqueAt(Units.Amps.of(3.0)).in(Units.NewtonMeters), 1e-12);
        assertEquals(freeSpeed, motor.expectedUnloadedSpeed(Units.Volts.of(12.0)).in(Units.RadiansPerSecond), 1e-9);
    }

    @Test
    void powerProfileTypedAccessorsReturnTheDeclaredValues() {
        PowerProfile profile = PowerProfile.of()
                .sustained(Units.Amps.of(60.0))
                .burst(Units.Amps.of(90.0), Units.Seconds.of(0.5))
                .rampLimit(Units.Amps.of(10.0), Units.Seconds.of(0.1));

        assertEquals(60.0, profile.sustainedCurrent().in(Units.Amps), 1e-9);
        assertEquals(90.0, profile.burstCurrent().in(Units.Amps), 1e-9);
        assertEquals(0.5, profile.burstDuration().in(Units.Seconds), 1e-9);
        assertEquals(10.0, profile.rampStep().in(Units.Amps), 1e-9);
        assertEquals(0.1, profile.rampInterval().in(Units.Seconds), 1e-9);
        assertEquals(100.0, profile.rampAmpsPerSecond(), 1e-9);
    }
}
